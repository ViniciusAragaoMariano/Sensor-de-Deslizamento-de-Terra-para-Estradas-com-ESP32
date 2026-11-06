# 🌍 Sistema de Monitoramento de Deslizamento de Terra para Estradas com ESP32

> Sistema embarcado de baixo custo para prevenção e monitoramento de riscos em encostas, combinando aquisição de dados em tempo real, interface local (OLED) e monitoramento remoto via página web.

---

## 📌 Sobre o Projeto

Este projeto desenvolve uma solução ciberfísica voltada à segurança viária. Utilizando o microcontrolador **ESP32**, o sistema capta dados críticos de movimentação do solo por meio de um acelerômetro e giroscópio (**MPU-6050**), além de monitorar variáveis ambientais com o sensor de temperatura e umidade **DHT11**. 

As informações processadas são exibidas localmente em uma tela **OLED** e disponibilizadas simultaneamente em uma **página web** hospedada pelo próprio ESP32, permitindo o acesso rápido e descentralizado aos dados.

## 🛠️ Tecnologias e Componentes

* **Hardware:**
  * Microcontrolador ESP32 (com Wi-Fi integrado)
  * Acelerômetro e Giroscópio MPU-6050 (detecção de inclinação/vibração)
  * Sensor de Temperatura e Umidade DHT11
  * Display OLED (I2C) para feedback visual local
* **Software / Comunicação:**
  * Servidor HTTP embarcado no ESP32
  * Protocolos de rede local para transmissão de telemetria
