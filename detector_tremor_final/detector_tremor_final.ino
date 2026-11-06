/*
  DETECTOR DE TREMORES - Sistema Ciberfisico (ESP32)
  Sensores:  MPU-6050 (tremor) e DHT11 (temperatura/umidade)
  Atuador:   Display OLED SSD1306 (controlado pela rede)

  Arquitetura: ESP32 = SERVIDOR HTTP (TCP, porta 80). Clientes (navegador, curl...) conectam via Wi-Fi.

  Rotas HTTP:
    GET /          pagina web (painel)
    GET /dados     dados dos sensores em tempo real (JSON)
    GET /status    IP, MAC, status do Wi-Fi, hora, uptime... (JSON)
    GET /log       log persistente da flash   (/log?n=30 = ultimas 30 linhas)
    GET /cmd?...   comandos ao dispositivo:
                     oled=on|off        liga/desliga o display
                     msg=texto          mostra mensagem no OLED por 15 s (vazio = cancela)
                     inverter=1|0       inverte as cores do OLED
                     leve=0.03&forte=0.10   ajusta os limiares
                     limparlog=1        apaga o log
    GET /hora?t=EPOCH   ajusta o relogio (usado pelo navegador quando nao ha NTP)

  Ligacoes:
    MPU-6050  VCC->3V3 | GND->GND | SDA->D21 | SCL->D22
    OLED      VCC->3V3 | GND->GND | SDA->D21 | SCL->D22
    DHT11     VCC->3V3 | GND->GND | DATA->D18

  Bibliotecas: Adafruit SSD1306, Adafruit GFX, DHT sensor library, Adafruit Unified Sensor.
  (WiFi, WebServer, LittleFS e Wire ja vem com o pacote ESP32.)
  Credenciais do Wi-Fi: arquivo segredos.h (mesma pasta do sketch).
*/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <time.h>
#include <sys/time.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include "segredos.h"

// ---------- Rede ----------
const bool  AP_RESERVA = true;                 // se nao conectar no Wi-Fi, cria rede propria de reserva
const char* AP_SSID    = "DetectorTremor";
const char* AP_SENHA   = "12345678";
const unsigned long TEMPO_CONEXAO_INICIAL = 15000;   // ms
const unsigned long INTERVALO_RECONEXAO   = 10000;   // ms entre tentativas

WebServer server(80);                          // porta TCP 80 (HTTP)

// ---------- Pinos ----------
#define SDA_PIN 21
#define SCL_PIN 22
#define DHT_PIN 18
#define DHT_TIPO DHT11

const uint8_t MPU_ADDR  = 0x68;
const uint8_t OLED_ADDR = 0x3C;

DHT dht(DHT_PIN, DHT_TIPO);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false;

// ---------- Log em flash ----------
const char* ARQ_LOG = "/log.txt";
const size_t LOG_MAX = 24 * 1024;              // ao passar disso, descarta a metade mais antiga
bool fsOk = false;

// ---------- Deteccao ----------
const int   AMOSTRAS_JANELA = 50;              // 50 amostras a 100 Hz = 0,5 s
float limiarLeve  = 0.03;                      // ajustaveis por comando
float limiarForte = 0.10;
const float FREQ_MIN_HZ = 3.0;
const float FREQ_MAX_HZ = 14.0;
const bool  EXIGIR_FREQUENCIA = true;

// ---------- Estado ----------
float ultimoRms = 0, ultimaFreq = 0;
int   ultimoNivel = 0;
int   nivelAnterior = 0;
unsigned long eventosFortes = 0;
unsigned long ultimoLogForte = 0;
float temperatura = NAN, umidade = NAN;
unsigned long ultimaLeituraDHT = 0;
int   falhasDHT = 0;
bool  dhtComFalha = false;
int   falhasMPU = 0;
bool  mpuComFalha = false;

// ---------- OLED (atuador) ----------
bool   oledLigado = true;
bool   oledInvertido = false;
String mensagemOled = "";
unsigned long mensagemAte = 0;

// ---------- Wi-Fi ----------
bool  wifiEstavaConectado = false;
bool  jaConectouAntes = false;
bool  modoAPAtivo = false;
unsigned long quedas = 0, reconexoes = 0;
unsigned long inicioQueda = 0;
unsigned long ultimaTentativa = 0;
unsigned long tentativas = 0;
unsigned long ultimaVerificacao = 0;
bool  horaJaLogada = false;

// ---------- Processamento ----------
float baseline = 1.0;
bool  baselineIniciado = false;
float somaQuadrados = 0;
int   cruzamentos = 0;
int   contador = 0;
float hpAnterior = 0;
unsigned long proximaAmostra = 0;

// =====================================================================
//  HORA E LOG
// =====================================================================
bool horaValida() {
  return time(nullptr) > 1700000000;           // maior que ~nov/2023 = relogio ajustado
}

String agora() {
  if (horaValida()) {
    time_t t = time(nullptr);
    struct tm tm;
    localtime_r(&t, &tm);
    char buf[24];
    strftime(buf, sizeof(buf), "%d/%m/%Y %H:%M:%S", &tm);
    return String(buf);
  }
  return "boot+" + String(millis() / 1000) + "s";   // enquanto nao ha hora real
}

String limparTexto(String s, size_t maximo) {
  String r = "";
  for (size_t i = 0; i < s.length() && r.length() < maximo; i++) {
    char c = s[i];
    if (c == '"' || c == '\\') c = '\'';
    if (c >= 32) r += c;
    else if (c == '\n' || c == '\r' || c == '\t') r += ' ';
  }
  r.trim();
  return r;
}

void podarLog() {
  File f = LittleFS.open(ARQ_LOG, FILE_READ);
  if (!f) return;
  String todo = f.readString();
  f.close();
  int corte = todo.indexOf('\n', todo.length() / 2);
  String resto = (corte >= 0) ? todo.substring(corte + 1) : "";
  File w = LittleFS.open(ARQ_LOG, FILE_WRITE);   // reescreve so com a metade mais recente
  if (w) { w.print(resto); w.close(); }
}

// nivel: "INFO", "AVISO" ou "ERRO"
void registrarLog(const char* nivel, const String& msg) {
  String linha = agora() + " [" + nivel + "] " + msg;
  Serial.println(linha);
  if (!fsOk) return;
  File f = LittleFS.open(ARQ_LOG, FILE_APPEND);
  if (!f) return;
  f.println(linha);
  size_t tamanho = f.size();
  f.close();
  if (tamanho > LOG_MAX) podarLog();
}

String lerLogTail(int n) {
  if (!fsOk || !LittleFS.exists(ARQ_LOG)) return "(log vazio)\n";
  File f = LittleFS.open(ARQ_LOG, FILE_READ);
  String todo = f.readString();
  f.close();
  if (todo.length() == 0) return "(log vazio)\n";
  if (n <= 0) return todo;
  int i = todo.length() - 1;
  if (i >= 0 && todo[i] == '\n') i--;
  int linhas = 0;
  for (; i >= 0; i--) {
    if (todo[i] == '\n') {
      linhas++;
      if (linhas >= n) break;
    }
  }
  return todo.substring(i + 1);
}

// =====================================================================
//  PAGINA WEB
// =====================================================================
const char PAGINA[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Detector de Tremor</title>
<style>
  body{font-family:system-ui,Arial,sans-serif;margin:0;padding:14px;background:#111;color:#eee;text-align:center}
  h1{font-size:1.25rem;margin:4px 0 12px}
  h2{font-size:.95rem;margin:18px 0 8px;color:#bbb;text-align:left}
  #estado{font-size:2.2rem;font-weight:700;padding:16px;border-radius:14px;background:#1e7a3c;transition:background .2s}
  #estado.leve{background:#b8860b}
  #estado.forte{background:#b3261e}
  .linha{display:flex;gap:8px;margin:8px 0;flex-wrap:wrap}
  .cx{flex:1;min-width:90px;background:#1c1c1c;border-radius:12px;padding:10px}
  .cx small{display:block;color:#999;font-size:.72rem}
  .cx b{font-size:1.15rem;word-break:break-all}
  canvas{width:100%;max-width:640px;height:180px;background:#1c1c1c;border-radius:12px}
  button{background:#2b5fa8;color:#fff;border:0;border-radius:8px;padding:9px 12px;font-size:.9rem;margin:3px}
  button.verm{background:#8a2a24}
  input{background:#222;color:#eee;border:1px solid #444;border-radius:8px;padding:8px;font-size:.9rem;margin:3px}
  .painel{background:#1c1c1c;border-radius:12px;padding:10px;text-align:left}
  pre{background:#0a0a0a;color:#9fe29f;border-radius:10px;padding:10px;font-size:.72rem;text-align:left;overflow-x:auto;max-height:260px;overflow-y:auto;white-space:pre-wrap}
  #aviso{min-height:1.2em;color:#4fc3f7;font-size:.85rem;margin:6px 0}
  a{color:#4fc3f7}
</style>
</head>
<body>
<h1>Detector de Tremor &mdash; ESP32</h1>
<div id="estado">---</div>

<div class="linha">
  <div class="cx"><small>Intensidade (g)</small><b id="rms">-</b></div>
  <div class="cx"><small>Frequ&ecirc;ncia (Hz)</small><b id="freq">-</b></div>
  <div class="cx"><small>Eventos fortes</small><b id="ev">-</b></div>
</div>
<div class="linha">
  <div class="cx"><small>Temperatura (&deg;C)</small><b id="temp">-</b></div>
  <div class="cx"><small>Umidade (%)</small><b id="umid">-</b></div>
</div>
<canvas id="grafico" width="640" height="180"></canvas>

<h2>Conex&atilde;o</h2>
<div class="linha">
  <div class="cx"><small>Status Wi-Fi</small><b id="wifi">-</b></div>
  <div class="cx"><small>IP</small><b id="ip">-</b></div>
  <div class="cx"><small>MAC</small><b id="mac">-</b></div>
</div>
<div class="linha">
  <div class="cx"><small>Sinal (dBm)</small><b id="rssi">-</b></div>
  <div class="cx"><small>Quedas / Reconex&otilde;es</small><b id="qr">-</b></div>
  <div class="cx"><small>Hora do dispositivo</small><b id="hora">-</b></div>
  <div class="cx"><small>Uptime</small><b id="up">-</b></div>
</div>

<h2>Comandos (atuador: display OLED)</h2>
<div class="painel">
  <button onclick="cmd('oled=on')">OLED ligar</button>
  <button onclick="cmd('oled=off')">OLED desligar</button>
  <button onclick="cmd('inverter=1')">Inverter cores</button>
  <button onclick="cmd('inverter=0')">Cores normais</button>
  <div style="margin-top:8px">
    <input id="msg" maxlength="30" placeholder="Mensagem no OLED (sem acentos)">
    <button onclick="enviarMsg()">Enviar</button>
    <button onclick="cmd('msg=')">Limpar</button>
  </div>
  <div style="margin-top:8px">
    Limiar leve <input id="lv" size="5"> forte <input id="fo" size="5">
    <button onclick="aplicarLimiares()">Aplicar</button>
  </div>
</div>
<div id="aviso"></div>

<h2>Log do sistema (flash) &mdash; <a href="/log" target="_blank">ver completo</a></h2>
<pre id="log">carregando...</pre>
<button class="verm" onclick="if(confirm('Apagar o log?'))cmd('limparlog=1')">Limpar log</button>

<script>
const hist = [];
const MAX = 80;
let limLeve = 0.03, limForte = 0.10, sincronizou = false, preencheu = false;

async function jget(u){ const r = await fetch(u,{cache:'no-store'}); return r.json(); }
function $(id){ return document.getElementById(id); }

function aviso(t){ $('aviso').textContent = t; setTimeout(()=>{ $('aviso').textContent=''; }, 5000); }

async function cmd(q){
  try{ const d = await jget('/cmd?'+q); aviso(d.msg); atualizarLog(); }
  catch(e){ aviso('falha ao enviar comando'); }
}
function enviarMsg(){ cmd('msg='+encodeURIComponent($('msg').value)); }
function aplicarLimiares(){ cmd('leve='+$('lv').value+'&forte='+$('fo').value); }

function desenhar(){
  const c = $('grafico'), g = c.getContext('2d');
  const W = c.width, H = c.height;
  g.clearRect(0,0,W,H);
  const topo = Math.max(limForte*2, ...hist, 0.01);
  const y = v => H - 8 - (v/topo)*(H-16);
  g.setLineDash([6,4]); g.lineWidth = 1;
  g.strokeStyle = '#b8860b'; g.beginPath(); g.moveTo(0,y(limLeve)); g.lineTo(W,y(limLeve)); g.stroke();
  g.strokeStyle = '#b3261e'; g.beginPath(); g.moveTo(0,y(limForte)); g.lineTo(W,y(limForte)); g.stroke();
  g.setLineDash([]);
  g.strokeStyle = '#4fc3f7'; g.lineWidth = 2; g.beginPath();
  hist.forEach((v,i)=>{ const x = (i/(MAX-1))*W; i ? g.lineTo(x,y(v)) : g.moveTo(x,y(v)); });
  g.stroke();
}

async function atualizar(){
  try{
    const d = await jget('/dados');
    limLeve = d.limLeve; limForte = d.limForte;
    const el = $('estado');
    el.textContent = d.estado;
    el.className = d.nivel === 2 ? 'forte' : d.nivel === 1 ? 'leve' : '';
    $('rms').textContent  = d.rms.toFixed(3);
    $('freq').textContent = d.freq.toFixed(1);
    $('ev').textContent   = d.eventos;
    $('temp').textContent = d.temp === null ? '-' : d.temp.toFixed(1);
    $('umid').textContent = d.umid === null ? '-' : d.umid.toFixed(0);
    hist.push(d.rms); if (hist.length > MAX) hist.shift();
    desenhar();
    if (!d.horaOk && !sincronizou){
      sincronizou = true;
      fetch('/hora?t=' + Math.floor(Date.now()/1000));
    }
  }catch(e){ $('estado').textContent = 'sem conexao...'; $('estado').className = ''; }
}

async function atualizarStatus(){
  try{
    const s = await jget('/status');
    $('wifi').textContent = s.wifi;
    $('ip').textContent   = s.ip;
    $('mac').textContent  = s.mac;
    $('rssi').textContent = s.rssi;
    $('qr').textContent   = s.quedas + ' / ' + s.reconexoes;
    $('hora').textContent = s.hora;
    $('up').textContent   = s.uptime + ' s';
    if (!preencheu){ $('lv').value = s.leve; $('fo').value = s.forte; preencheu = true; }
  }catch(e){}
}

async function atualizarLog(){
  try{
    const r = await fetch('/log?n=25', {cache:'no-store'});
    const t = await r.text();
    const p = $('log'); p.textContent = t; p.scrollTop = p.scrollHeight;
  }catch(e){}
}

setInterval(atualizar, 500);
setInterval(atualizarStatus, 3000);
setInterval(atualizarLog, 5000);
atualizar(); atualizarStatus(); atualizarLog();
</script>
</body>
</html>
)rawliteral";

// =====================================================================
//  ROTAS HTTP
// =====================================================================
void rotaPagina() {
  server.send_P(200, "text/html; charset=utf-8", PAGINA);
}

void rotaDados() {
  const char* texto = (ultimoNivel == 0) ? "PARADO" : (ultimoNivel == 1) ? "LEVE" : "FORTE";
  char sTemp[12], sUmid[12];
  if (isnan(temperatura)) strcpy(sTemp, "null"); else snprintf(sTemp, sizeof(sTemp), "%.1f", temperatura);
  if (isnan(umidade))     strcpy(sUmid, "null"); else snprintf(sUmid, sizeof(sUmid), "%.0f", umidade);

  char json[360];
  snprintf(json, sizeof(json),
           "{\"rms\":%.4f,\"freq\":%.1f,\"nivel\":%d,\"estado\":\"%s\","
           "\"eventos\":%lu,\"limLeve\":%.3f,\"limForte\":%.3f,"
           "\"temp\":%s,\"umid\":%s,\"horaOk\":%s}",
           ultimoRms, ultimaFreq, ultimoNivel, texto,
           eventosFortes, limiarLeve, limiarForte, sTemp, sUmid,
           horaValida() ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void rotaStatus() {
  bool conectado = (WiFi.status() == WL_CONNECTED);
  String ip = conectado ? WiFi.localIP().toString() : (modoAPAtivo ? WiFi.softAPIP().toString() : "-");
  String json = "{";
  json += "\"wifi\":\"" + String(conectado ? "CONECTADO" : "DESCONECTADO") + "\",";
  json += "\"ssid\":\"" + limparTexto(WiFi.SSID(), 32) + "\",";
  json += "\"ip\":\"" + ip + "\",";
  json += "\"mac\":\"" + WiFi.macAddress() + "\",";
  json += "\"rssi\":" + String(conectado ? WiFi.RSSI() : 0) + ",";
  json += "\"modoAP\":" + String(modoAPAtivo ? "true" : "false") + ",";
  json += "\"quedas\":" + String(quedas) + ",";
  json += "\"reconexoes\":" + String(reconexoes) + ",";
  json += "\"hora\":\"" + agora() + "\",";
  json += "\"horaOk\":" + String(horaValida() ? "true" : "false") + ",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"oled\":" + String(oledLigado ? "true" : "false") + ",";
  json += "\"leve\":" + String(limiarLeve, 3) + ",";
  json += "\"forte\":" + String(limiarForte, 3);
  json += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void rotaLog() {
  int n = server.hasArg("n") ? server.arg("n").toInt() : 0;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain; charset=utf-8", lerLogTail(n));
}

void rotaHora() {
  if (server.hasArg("t")) {
    long t = server.arg("t").toInt();
    if (t > 1700000000 && !horaValida()) {
      struct timeval tv = { (time_t)t, 0 };
      settimeofday(&tv, nullptr);
      server.send(200, "text/plain", "hora ajustada");
      return;
    }
  }
  server.send(200, "text/plain", "hora nao alterada");
}

void responder(bool ok, const String& msg) {
  String json = "{\"ok\":" + String(ok ? "true" : "false") + ",\"msg\":\"" + limparTexto(msg, 120) + "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(ok ? 200 : 400, "application/json", json);
}

void mostrarOLED();   // declarada adiante

void rotaCmd() {
  String origem = server.client().remoteIP().toString();
  String resp = "";
  bool recebeu = false;

  if (server.hasArg("oled")) {
    String v = server.arg("oled");
    recebeu = true;
    if (v == "on" || v == "1") {
      oledLigado = true;
      if (oledOk) display.ssd1306_command(SSD1306_DISPLAYON);
      resp += "OLED ligado; ";
      registrarLog("INFO", "Comando de " + origem + ": OLED ligado");
    } else if (v == "off" || v == "0") {
      oledLigado = false;
      if (oledOk) display.ssd1306_command(SSD1306_DISPLAYOFF);
      resp += "OLED desligado; ";
      registrarLog("INFO", "Comando de " + origem + ": OLED desligado");
    } else {
      responder(false, "valor invalido para oled (use on ou off)");
      return;
    }
  }

  if (server.hasArg("inverter")) {
    String v = server.arg("inverter");
    recebeu = true;
    oledInvertido = (v == "1" || v == "on");
    if (oledOk) display.invertDisplay(oledInvertido);
    resp += oledInvertido ? "cores invertidas; " : "cores normais; ";
    registrarLog("INFO", "Comando de " + origem + ": inverter=" + String(oledInvertido ? 1 : 0));
  }

  if (server.hasArg("msg")) {
    String m = limparTexto(server.arg("msg"), 30);
    recebeu = true;
    if (m.length() == 0) {
      mensagemOled = "";
      resp += "mensagem removida; ";
      registrarLog("INFO", "Comando de " + origem + ": mensagem do OLED removida");
    } else {
      mensagemOled = m;
      mensagemAte = millis() + 15000;
      resp += "mensagem enviada ao OLED; ";
      registrarLog("INFO", "Comando de " + origem + ": mensagem no OLED = " + m);
    }
  }

  if (server.hasArg("leve") || server.hasArg("forte")) {
    recebeu = true;
    float novoLeve  = server.hasArg("leve")  ? server.arg("leve").toFloat()  : limiarLeve;
    float novoForte = server.hasArg("forte") ? server.arg("forte").toFloat() : limiarForte;
    if (novoLeve < 0.001 || novoForte > 2.0 || novoLeve >= novoForte) {
      responder(false, "limiares invalidos (0.001 <= leve < forte <= 2.0)");
      return;
    }
    limiarLeve = novoLeve;
    limiarForte = novoForte;
    resp += "limiares atualizados; ";
    registrarLog("INFO", "Comando de " + origem + ": limiares leve=" + String(limiarLeve, 3) +
                         " forte=" + String(limiarForte, 3));
  }

  if (server.hasArg("limparlog")) {
    recebeu = true;
    if (fsOk) LittleFS.remove(ARQ_LOG);
    registrarLog("AVISO", "Log apagado por comando de " + origem);
    resp += "log apagado; ";
  }

  if (!recebeu) {
    responder(false, "nenhum comando reconhecido (oled, msg, inverter, leve, forte, limparlog)");
    return;
  }
  mostrarOLED();
  responder(true, resp);
}

// =====================================================================
//  MPU-6050 / DHT11 / OLED
// =====================================================================
void escreverReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool lerAceleracao(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / 16384.0;
  ay = ry / 16384.0;
  az = rz / 16384.0;
  return true;
}

void atualizarDHT() {
  if (millis() - ultimaLeituraDHT < 2000) return;
  ultimaLeituraDHT = millis();
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (isnan(t) || isnan(h)) {
    falhasDHT++;
    if (falhasDHT >= 5 && !dhtComFalha) {
      dhtComFalha = true;
      registrarLog("AVISO", "DHT11 sem leitura valida (verifique fiacao)");
    }
  } else {
    temperatura = t;
    umidade = h;
    falhasDHT = 0;
    if (dhtComFalha) {
      dhtComFalha = false;
      registrarLog("INFO", "DHT11 voltou a responder");
    }
  }
}

void mostrarOLED() {
  if (!oledOk || !oledLigado) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (mensagemOled.length() > 0 && millis() < mensagemAte) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("MENSAGEM:");
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(mensagemOled);
    display.display();
    return;
  }
  mensagemOled = "";

  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(ultimoNivel == 0 ? "PARADO" : ultimoNivel == 1 ? "LEVE" : "FORTE!");

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.printf("Int:%.3fg F:%.1fHz", ultimoRms, ultimaFreq);

  display.setCursor(0, 31);
  if (isnan(temperatura) || isnan(umidade)) display.print("DHT: sem leitura");
  else display.printf("T:%.1fC  U:%.0f%%", temperatura, umidade);

  bool conectado = (WiFi.status() == WL_CONNECTED);
  display.setCursor(0, 42);
  if (conectado) display.printf("WiFi OK %ddBm", WiFi.RSSI());
  else display.print(modoAPAtivo ? "WiFi OFF (AP ativo)" : "WiFi desconectado");

  display.setCursor(0, 54);
  if (conectado) display.print(WiFi.localIP());
  else if (modoAPAtivo) display.print(WiFi.softAPIP());
  display.display();
}

// =====================================================================
//  Wi-Fi: conexao, monitoramento e reconexao com log
// =====================================================================
void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  Serial.print("Conectando ao Wi-Fi");
  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < TEMPO_CONEXAO_INICIAL) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED && AP_RESERVA) {
    WiFi.mode(WIFI_AP_STA);                // rede de reserva + continua tentando a principal
    WiFi.softAP(AP_SSID, AP_SENHA);
    WiFi.begin(WIFI_SSID, WIFI_SENHA);
    modoAPAtivo = true;
  }
}

void verificarWiFi(bool forcar) {
  if (!forcar && millis() - ultimaVerificacao < 2000) return;
  ultimaVerificacao = millis();

  bool conectado = (WiFi.status() == WL_CONNECTED);

  if (conectado && !wifiEstavaConectado) {
    wifiEstavaConectado = true;
    if (jaConectouAntes) {
      reconexoes++;
      unsigned long fora = (millis() - inicioQueda) / 1000;
      registrarLog("INFO", "Wi-Fi RECONECTADO apos " + String(fora) + " s. IP=" +
                           WiFi.localIP().toString() + " RSSI=" + String(WiFi.RSSI()) + " dBm");
    } else {
      registrarLog("INFO", "Wi-Fi conectado em '" + limparTexto(WiFi.SSID(), 32) + "'. IP=" +
                           WiFi.localIP().toString() + " MAC=" + WiFi.macAddress() +
                           " RSSI=" + String(WiFi.RSSI()) + " dBm");
    }
    jaConectouAntes = true;
    tentativas = 0;
  }
  else if (!conectado) {
    if (wifiEstavaConectado) {                       // acabou de cair
      wifiEstavaConectado = false;
      quedas++;
      inicioQueda = millis();
      registrarLog("AVISO", "Conexao Wi-Fi PERDIDA (queda #" + String(quedas) + ")");
    } else if (!jaConectouAntes && tentativas == 0 && forcar) {
      registrarLog("ERRO", "Nao foi possivel conectar ao Wi-Fi '" + String(WIFI_SSID) +
                           "'. " + String(modoAPAtivo ? "Rede de reserva ativa em 192.168.4.1" : ""));
    }
    if (millis() - ultimaTentativa > INTERVALO_RECONEXAO) {
      ultimaTentativa = millis();
      tentativas++;
      WiFi.reconnect();
      if (tentativas % 6 == 1 && tentativas > 1) {   // evita encher o log
        registrarLog("AVISO", "Ainda sem Wi-Fi. Tentativa de reconexao #" + String(tentativas));
      }
    }
  }
}

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=== DETECTOR DE TREMOR - SISTEMA CIBERFISICO ===");

  fsOk = LittleFS.begin(true);                     // true = formata na primeira vez
  if (!fsOk) Serial.println("ERRO: LittleFS nao iniciou (log so no Serial).");
  registrarLog("INFO", "Sistema iniciado");

  Wire.begin(SDA_PIN, SCL_PIN);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Wire.setClock(400000);
  if (oledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Detector de tremor");
    display.println("Conectando ao WiFi...");
    display.display();
  } else {
    registrarLog("AVISO", "OLED nao encontrado (endereco 0x3C)");
  }

  dht.begin();

  // MPU-6050
  escreverReg(0x6B, 0x80);
  delay(150);
  escreverReg(0x6B, 0x01);
  delay(100);
  escreverReg(0x6C, 0x00);
  escreverReg(0x1C, 0x00);
  escreverReg(0x19, 0x07);
  escreverReg(0x1A, 0x03);
  delay(100);

  float ax, ay, az;
  if (lerAceleracao(ax, ay, az)) {
    registrarLog("INFO", "MPU-6050 iniciado");
  } else {
    mpuComFalha = true;
    registrarLog("ERRO", "MPU-6050 nao respondeu na inicializacao");
  }

  // Wi-Fi e hora (fuso de Brasilia, UTC-3). O NTP so funciona se a rede tiver internet.
  iniciarWiFi();
  configTzTime("<-03>3", "pool.ntp.org", "time.google.com");

  server.on("/", rotaPagina);
  server.on("/dados", rotaDados);
  server.on("/status", rotaStatus);
  server.on("/log", rotaLog);
  server.on("/cmd", rotaCmd);
  server.on("/hora", rotaHora);
  server.begin();
  verificarWiFi(true);
  registrarLog("INFO", "Servidor HTTP iniciado na porta 80");

  atualizarDHT();
  mostrarOLED();
  proximaAmostra = micros();
}

void loop() {
  server.handleClient();
  verificarWiFi(false);

  if (!horaJaLogada && horaValida()) {
    horaJaLogada = true;
    registrarLog("INFO", "Relogio sincronizado");
  }

  // Amostragem a 100 Hz (10 ms)
  if ((long)(micros() - proximaAmostra) < 0) return;
  proximaAmostra += 10000;

  float ax, ay, az;
  if (!lerAceleracao(ax, ay, az)) {
    falhasMPU++;
    if (falhasMPU == 100 && !mpuComFalha) {          // ~1 s seguido sem leitura
      mpuComFalha = true;
      registrarLog("ERRO", "MPU-6050 sem resposta (verifique SDA/SCL)");
    }
    return;
  }
  falhasMPU = 0;
  if (mpuComFalha) {
    mpuComFalha = false;
    registrarLog("INFO", "MPU-6050 voltou a responder");
  }

  float mag = sqrt(ax * ax + ay * ay + az * az);
  if (!baselineIniciado) {
    baseline = mag;
    baselineIniciado = true;
  }
  baseline = baseline * 0.95 + mag * 0.05;           // remove a gravidade
  float hp = mag - baseline;

  somaQuadrados += hp * hp;
  if ((hp > 0 && hpAnterior < 0) || (hp < 0 && hpAnterior > 0)) cruzamentos++;
  hpAnterior = hp;
  contador++;

  if (contador >= AMOSTRAS_JANELA) {
    float rms  = sqrt(somaQuadrados / contador);
    float dur  = contador * 0.01;
    float freq = (cruzamentos / 2.0) / dur;

    bool freqOk = !EXIGIR_FREQUENCIA || (freq >= FREQ_MIN_HZ && freq <= FREQ_MAX_HZ);
    int nivel = 0;
    if (rms >= limiarForte && freqOk)      nivel = 2;
    else if (rms >= limiarLeve && freqOk)  nivel = 1;

    ultimoRms   = rms;
    ultimaFreq  = freq;
    ultimoNivel = nivel;

    // Registra apenas as TRANSICOES (evita gastar a flash)
    if (nivel == 2 && nivelAnterior != 2) {
      eventosFortes++;
      if (millis() - ultimoLogForte > 5000 || ultimoLogForte == 0) {
        ultimoLogForte = millis();
        registrarLog("AVISO", "Tremor FORTE detectado (RMS=" + String(rms, 3) +
                              " g, f=" + String(freq, 1) + " Hz)");
      }
    } else if (nivel != 2 && nivelAnterior == 2) {
      registrarLog("INFO", "Tremor forte encerrado");
    }
    nivelAnterior = nivel;

    atualizarDHT();
    mostrarOLED();

    Serial.printf("%.4f | %.1f | %s | T=%.1f U=%.0f\n", rms, freq,
                  nivel == 0 ? "PARADO" : nivel == 1 ? "LEVE" : "FORTE",
                  temperatura, umidade);

    somaQuadrados = 0;
    cruzamentos = 0;
    contador = 0;
    proximaAmostra = micros();
  }
}
