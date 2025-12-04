# ChairPosture-IoT-
Projeto de protótipo IoT com ESP32 + MPU6050 + FSR que detecta quando uma pessoa está sentada e se a sua postura é ruim (inclinação para frente / lateral). O ESP32 publica telemetria e eventos via MQTT para um broker (Node-RED consome e mostra Dashboard).

# Índice
* [Visão Geral](#visão-geral)
* [Componentes e Ferramentas](#componentes-e-ferramentas)
* [Diagrama / Wiring (esquemático)](#diagrama--wiring-esquemático)
* [Firmware (explicação e estrutura)](#firmware-explicação-e-estrutura)
* [Configuração recomendada](#configuração-recomendada)
* [Tópicos MQTT e payloads](#tópicos-mqtt-e-payloads)
* [Node-RED — Dashboard & fluxo (Mute Alerts)](#node-red--dashboard--fluxo-mute-alerts)
* [Como compilar e gravar (Arduino IDE / PlatformIO)](#como-compilar-e-gravar-arduino-ide--platformio)
* [Testes e verificação](#testes-e-verificação)
* [Troubleshooting rápido](#troubleshooting-rápido)
* [Parâmetros visuais recomendados (gauges, charts, thresholds)](#parâmetros-visuais-recomendados-gauges-charts-thresholds)
* [Boas práticas de segurança](#boas-práticas-de-segurança)
* [Melhorias futuras (priorizadas)](#melhorias-futuras-priorizadas)

---
## Visão-geral 

O sistema detecta:

- Presença: via FSR (Force Sensitive Resistor) no assento.
- Orientação: pitch e roll via MPU6050 (acelerômetro).
- Classificação: absent, good, bad (bad quando inclinação excede thresholds).
- Notificação: publica evento posture_bad quando má postura persistir N leituras.

Fluxo end-to-end:
ESP32 → MQTT → Node-RED → Dashboard (gráficos + toast alerts).
Botão Mute Alerts no dashboard bloqueia exibição de toasts mantendo o log.

## Componentes e Ferramentas

Hardware

- ESP32 dev board (ex.: WROOM / DOIT)
- MPU6050 (I²C accel + gyro)
- FSR (Force Sensing Resistor) pad
- Resistor fixo para divisor (recomendado 10 kΩ)
- Cabos dupont / protoboard / fita dupla-face
- Fonte USB 5V para alimentar o ESP32

Software

- Arduino IDE ou PlatformIO
- Bibliotecas:
  - MPU6050 (Jeff Rowberg ou equivalente)
  - PubSubClient
  - Wire, WiFi (ESP32 core)
- Node-RED com node-red-dashboard
- Broker MQTT (ex.: broker.emqx.io para testes, ou Mosquitto local)

## Diagrama / Wiring (esquemático)

Conexões texto (simples)
- MPU6050
  - VCC → 3.3V (ESP32)
  - GND → GND (ESP32)
  - SDA → GPIO21 (SDA_PIN)
  - SCL → GPIO22 (SCL_PIN)

- FSR (divisor)
  - FSR entre 3.3V e nó A
  - Resistência fixa 10kΩ entre nó A e GND
  - Nó A → GPIO36 (ADC1_CH0) do ESP32 (FSR_PIN)
 
```lua
ESP32
+-----+                MPU6050
|3.3V |---+--------------- VCC
|GND  |---+--------------- GND
|21(SDA)|---- SDA ------> SDA
|22(SCL)|---- SCL ------> SCL

FSR pad ----+--- node A ----> GPIO36 (ADC1_CH0)
            |
           3.3V
node A ---- 10kΩ ---- GND

```
Notas elétricas
- MPU6050 = 3.3V (não ligar a 5V);
- Use pull-ups de I²C se cabo longo (>20cm), 4.7kΩ entre SDA/SCL e 3.3V;
- ADC do ESP32 configurado com ADC_11db (atenuação para ~3.3V).

## Firmware (explicação e estrutura)

Abaixo está o esqueleto principal do firmware usado no protótipo
Arquivo principal pode ser encontrado em: **`sketch_dec3a_copy_20251204115311.ino`**

setup()

- Serial.begin(115200) para debug.
- I²C Wire.begin(SDA, SCL).
- Inicializa MPU6050 e verifica conexão (mpu.testConnection()).
- Configura ADC (analogReadResolution(12) e analogSetPinAttenuation(FSR_PIN, ADC_11db)).
- Conecta WiFi (connectWiFi()), configura MQTT (mqttClient.setServer()).
- Mensagem: enviar 'c' no Serial para calibrar.
  
loop()

- Reconnect WiFi/MQTT se necessário.
- mqttClient.loop() para manter a conexão e callbacks.
- Lê FSR (analogRead(FSR_PIN) → mapeia 0..4095 para 0..100%).
- Define presence se fsrPercent >= PRESENCE_PERCENT_THRESHOLD.
- Se presence, lê acelerômetro (mpu.getAcceleration) e calcula pitch e roll:

```cpp
pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
roll  = atan2(ay, az) * 180.0 / PI;
```

- Ajusta offsets se calibrado.
- Determina state: "absent", "good", "bad".
- Publica telemetria (publishTelemetry) no tópico chairposture/{device}/telemetry.
- Se state == bad incrementa consecutiveBadPosture e publica posture_bad quando atinge PERSISTENCE_COUNT.
- Captura c via Serial para calibrateBaseline().

publishTelemetry(...)

- Monta JSON com device_id, ts (millis), pitch, roll, fsr_raw, fsr_percent, posture_state.
- Publica para chairposture/{DEVICE_ID}/telemetry.

publishEvent(...)

- Monta JSON evento com event e severity, publica em chairposture/{DEVICE_ID}/event.

calibrateBaseline()

- Lê SAMPLES amostras (20), calcula média pitch/roll e salva offsets.
- OBS: função é bloqueante (usa delay) — há risco de desconexão MQTT durante calibração. Para protótipo isto é aceitável; em produção mude para versão não-blocante.

## Configuração recomendada
```cpp
// config.example.h  (copy -> config.h and fill, then add config.h to .gitignore)
#pragma once

const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASS";

const char* MQTT_SERVER = "broker.example.com";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USER = "";       // empty if not used
const char* MQTT_PASS = "";

const char* DEVICE_ID = "chair001";
```

## Tópicos MQTT e payloads

Tópicos

- Telemetria: chairposture/{device_id}/telemetry (QoS 0)
- Eventos: chairposture/{device_id}/event (QoS 1)
  
Exemplo payload telemetry
```json
{
  "device_id":"chair001",
  "ts":499871,
  "pitch":3.12,
  "roll":-1.27,
  "fsr_raw":2048,
  "fsr_percent":50,
  "posture_state":"good"
}
```
Exemplo payload event

```json
{
  "device_id":"chair001",
  "ts":509871,
  "event":"posture_bad",
  "severity":"warning"
}
```

## Node-RED — Dashboard & fluxo (Mute Alerts)

Objetivo

- Mostrar telemetria (pitch/roll/FSR)
- Mostrar gráfico tempo real (pitch, roll)
- Mostrar notificação (toast) quando posture_bad
- Botão Mute Alerts para bloquear toasts

Estrutura principal do fluxo

1. mqtt in (telemetry) → json → Split telemetry → charts / gauges / texts
2. mqtt in (event) → json → Format event → Check mute → ui_toast
3. ui_button (Mute Alerts) → set-mute (function) → toggles flow.set("muteAlerts", true/false) e atualiza label

Funções

Split telemetry (Function)

```javascript
// expects msg.payload as telemetry JSON
var t = msg.payload || {};
var pitch = parseFloat(t.pitch) || 0;
var roll = parseFloat(t.roll) || 0;
var fsr = parseInt(t.fsr_percent || t.fsrPercent || t.fsr_raw || 0);
var state = (t.posture_state || t.posture || t.state || "unknown").toString();
// outputs: [pitch, roll, fsr, state]
return [{payload: pitch},{payload: roll},{payload: fsr},{payload: state}];
```

Format event → user-friendly text (Function)
```javascript
var p = msg.payload;
if (typeof p === "string") {
  try { p = JSON.parse(p); } catch(e){ p = {}; }
}
var device = p.device_id || p.device || "dispositivo";
var ev = (p.event || p.type || "").toString();
var sev = (p.severity || "").toString();
var ts = p.ts || 0;

var label = ev || "evento";
if (ev === "posture_bad") label = "Postura ruim detectada";
else if (ev === "posture_good") label = "Postura corrigida";
else if (ev === "calibration") label = "Calibração";

var timeStr = "";
if (ts && ts > 1_000_000_000_000) {
  timeStr = new Date(ts).toLocaleString();
} else if (ts && ts > 1_000_000_000) {
  timeStr = new Date(ts*1000).toLocaleString();
} else if (ts && ts > 0) {
  var s = Math.floor(ts / 1000);
  var m = Math.floor(s / 60);
  var sec = s % 60;
  timeStr = m + "m " + sec + "s (uptime)";
}

var parts = [];
parts.push(label);
parts.push(device);
if (sev) parts.push("gravidade: " + sev);
if (timeStr) parts.push(timeStr);

msg.payload = parts.join(" — ");
msg.topic = "ALERTA";
msg.displayTime = 8000;
return msg;
```

set-mute (Function) — toggles mute and updates button label

```javascript
var current = flow.get("muteAlerts") || false;
var newState = !current;
flow.set("muteAlerts", newState);
// update button label visually
msg.ui_control = { label: newState ? "Unmute Alerts" : "Mute Alerts" };
node.status({ fill: newState ? "red" : "green", shape: "dot", text: newState ? "Muted" : "Unmuted" });
return msg;
```

Check mute (Function) — place between Format event and ui_toast

```javascript
var muted = flow.get("muteAlerts") || false;
if (muted) return null; // blocks toast
return msg; // allow toast
```

Como importar o fluxo

1. Abra Node-RED → Menu → Import → Clipboard.
2. Cole o JSON do fluxo (se tiver exportado) ou crie nodes conforme os códigos acima.
3. Configure o node mqtt-broker com o host do broker.
4. Salve e abra o dashboard (http://<NODE_RED_HOST>:1880/ui).

## Como compilar e gravar (Arduino IDE / PlatformIO)

Arduino IDE

1. Instale suporte ao ESP32 via Board Manager (URL Espressif).
2. Instale bibliotecas MPU6050 e PubSubClient pelo Library Manager.
3. Abra chair_posture.ino e inclua config.h com suas credenciais (não comitar).
4. Selecione a placa (ex.: ESP32 Dev Module) e a porta serial.
5. Compile e Upload.

PlatformIO

platformio.ini mínimo:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
  pjrc/MPU6050@^1.0.0   ; ajuste conforme lib que usar
  knolleary/PubSubClient@^2.8
```

Depois: pio run -t upload

## Testes e verificação

Teste rápido

1. Abra Serial Monitor 115200 bps. Deve mostrar inicialização (MPU conectado, WiFi conectado).
2. Pressione c no Serial para calibrar — verá pitchOffset e rollOffset.
3. Sente na cadeira: verifique mensagens PUB telemetry -> chairposture/chair001/telemetry ....
4. Incline-se até ultrapassar thresholds; confirme que evento posture_bad é publicado após PERSISTENCE_COUNT leituras.

No Node-RED

- Verifique gauges e gráficos atualizando;
- Gere um evento (posture_bad) e observe o ui_toast (a menos que Mute esteja ativo);
- Teste o botão Mute Alerts: com mute ON, toasts não aparecem.

## Troubleshooting rápido

- MQTT publish retorna false
  - Verifique MQTT_MAX_PACKET_SIZE do PubSubClient se payload grande; ou reduza payload.
  - Verifique broker, porta, credenciais e alcance de rede.
- MPU6050 NAO conectado
  - Confirme ligações SDA/SCL/VCC/GND; execute scanner I²C; tente outra biblioteca.
- FSR leitura errada (0 ou saturado)
  - Verifique divisor (resistor), faça “multimeter” no nó A; ajuste resistor (4.7k–47k) conforme sensibilidade.
- Node-RED mostra JSON cru em toast
  - Garanta que o json node está com action = obj antes da função de formatação.
- Calibração desconecta MQTT
  - calibrateBaseline() é bloqueante; para evitar desconexão, chame mqttClient.loop() dentro do laço de amostras ou faça calibração não-bloqueante.

## Parâmetros visuais recomendados (gauges, charts, thresholds)

Pitch (°)
- Range (gauge / chart): -30 … +30 (ou -40..40 se quiser margem).
- Bands:
  - Verde: <= 8°
  - Amarelo: > 8° e <= 12°
  - Vermelho: > 12°
- Roll (°)
  - Range: -30 … +30
  - Thresholds baseados em |roll|:
    - Verde: <= 8°
    - Amarelo: 8..12°
    - Vermelho: > 12°
- FSR (%)
  - Range: 0 .. 100
  - Presence hysteresis:
    - Enter presence: >= 12%
    - Exit presence: <= 8%
- Chart window
  - Default: mostrar últimos 5–30 minutos.
  - removeOlder no node ui_chart para controlar retenção.

## Boas práticas de segurança

- Use broker com TLS (MQTTS 8883) e WiFiClientSecure em produção.
- Use LWT (Last Will) e tópico chairposture/{id}/state com retain para indicador online/offline.
- Proteja Node-RED com autenticação e HTTPS.

## Melhorias futuras (priorizadas)

1. Filtro Madgwick / Kalman para combinar gyro+accel e suavizar orientação.
2. Calibração não-bloqueante (para não interromper mqttClient.loop()).
3. OTA (ArduinoOTA / web update).
4. Autocalibração FSR (mapear min/max e ajustar thresholds dinamicamente).
5. Registro histórico (InfluxDB + Grafana ou CSV export via Node-RED).
6. Notificações com ACK (botão confirm e log).
7. Treinamento por usuário (aprender padrões e reduzir falsos positivos).
