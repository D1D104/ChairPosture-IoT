#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ================== CONFIG ==================
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

const char* MQTT_SERVER = ""; // broker IP ou hostname
const uint16_t MQTT_PORT = ;
const char* MQTT_USER = "";       // se não usar, deixe "" e MQTT_USER empty
const char* MQTT_PASS = "";

const char* DEVICE_ID = "chair001";

// Pinos
#define SDA_PIN 21
#define SCL_PIN 22
#define FSR_PIN 36   // ADC1_CH0 (GPIO36)

// Leitura / thresholds
const int PRESENCE_PERCENT_THRESHOLD = 10;   // % - acima disso considera "presença"
const float PITCH_FORWARD_THRESHOLD = 10.0;  // graus -> inclinação para frente
const float ROLL_THRESHOLD = 12.0;           // graus -> inclinação lateral
const int   PERSISTENCE_COUNT = 3;           // número de leituras consecutivas para confirmar evento

const unsigned long TELEMETRY_INTERVAL_MS = 5000; // publica telemetria a cada 5s se sentado
const unsigned long TELEMETRY_INTERVAL_NOSEAT_MS = 30000; // quando ausente, publica a cada 30s

// Conversão acelerômetro (±2g default -> 16384 LSB/g)
const float ACCEL_SENSITIVITY = 16384.0;

// ================ Globals / libs =================
MPU6050 mpu;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastTelemetry = 0;
bool calibrated = false;
float pitchOffset = 0.0;
float rollOffset = 0.0;

int consecutiveBadPosture = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\n=== Chair Posture MQTT test ===");

  // I2C and MPU
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if (mpu.testConnection()) Serial.println("MPU6050 conectado");
  else Serial.println("MPU6050 NAO conectado!");

  // ADC
  analogReadResolution(12); // 0..4095
  analogSetPinAttenuation(FSR_PIN, ADC_11db);

  // WiFi + MQTT
  WiFi.mode(WIFI_STA);
  connectWiFi();
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  Serial.println("Enviar 'c' no Serial para calibrar postura ereta (5s).");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqttClient.connected()) mqttReconnect();
  mqttClient.loop();

  // leitura FSR
  int fsrRaw = analogRead(FSR_PIN);               // 0..4095
  int fsrPercent = constrain(map(fsrRaw, 0, 4095, 0, 100), 0, 100);

  bool presence = (fsrPercent >= PRESENCE_PERCENT_THRESHOLD);

  // se presença, medir MPU; se não, enviar telemetria menos frequente
  unsigned long now = millis();
  unsigned long telemetryInterval = presence ? TELEMETRY_INTERVAL_MS : TELEMETRY_INTERVAL_NOSEAT_MS;
  if (now - lastTelemetry >= telemetryInterval) {
    lastTelemetry = now;

    float pitch = 0.0, roll = 0.0;
    if (presence) {
      // le MPu e calcula pitch/roll
      int16_t ax_raw, ay_raw, az_raw;
      mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
      float ax = (float)ax_raw / ACCEL_SENSITIVITY;
      float ay = (float)ay_raw / ACCEL_SENSITIVITY;
      float az = (float)az_raw / ACCEL_SENSITIVITY;

      // calcular pitch/roll simples
      pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
      roll  = atan2(ay, az) * 180.0 / PI;

      if (calibrated) {
        pitch -= pitchOffset;
        roll  -= rollOffset;
      }
    }

    // determinar estado de postura
    const char* state = "unknown";
    if (!presence) state = "absent";
    else {
      bool badPitch = (pitch > PITCH_FORWARD_THRESHOLD);
      bool badRoll = (fabs(roll) > ROLL_THRESHOLD);
      if (badPitch || badRoll) state = "bad";
      else state = "good";
    }

    // publish telemetry JSON
    publishTelemetry(pitch, roll, fsrRaw, fsrPercent, state);

    // se state == bad, acompanhar persistência e publicar evento quando confirmado
    if (presence && strcmp(state, "bad") == 0) {
      consecutiveBadPosture++;
      if (consecutiveBadPosture >= PERSISTENCE_COUNT) {
        // publica evento e reseta contador para evitar spam (pode rearmar depois)
        publishEvent("posture_bad", "warning");
        consecutiveBadPosture = 0;
      }
    } else {
      consecutiveBadPosture = 0;
    }
  }

  // checar input serial para calibrar
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') {
      calibrateBaseline();
    }
  }
  delay(50);
}

// ----- WiFi / MQTT helpers -----
void connectWiFi() {
  Serial.printf("Conectando WiFi: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) Serial.println("WiFi conectado");
  else Serial.println("Falha WiFi (verifique credenciais)");
}

void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.print("Conectando MQTT...");
  while (!mqttClient.connected()) {
    String clientId = String(DEVICE_ID) + "-" + String(random(0xffff), HEX);
    bool ok;
    if (strlen(MQTT_USER) == 0) ok = mqttClient.connect(clientId.c_str());
    else ok = mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);

    if (ok) {
      Serial.println(" conectado");
      // se quiser subscribes, coloque aqui
      // mqttClient.subscribe("chairposture/+/cmd");
    } else {
      Serial.print(" falha, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" tentando novamente em 2s");
      delay(2000);
    }
  }
}

// ----- Publish helpers -----
void publishTelemetry(float pitch, float roll, int fsrRaw, int fsrPercent, const char* state) {
  // montar JSON simples
  char topic[128];
  char payload[512];
  snprintf(topic, sizeof(topic), "chairposture/%s/telemetry", DEVICE_ID);

  unsigned long ts = millis();
  // payload: device_id, ts, pitch, roll, fsrRaw, fsrPercent, posture_state
  int len = snprintf(payload, sizeof(payload),
    "{\"device_id\":\"%s\",\"ts\":%lu,\"pitch\":%.2f,\"roll\":%.2f,\"fsr_raw\":%d,\"fsr_percent\":%d,\"posture_state\":\"%s\"}",
    DEVICE_ID, ts, pitch, roll, fsrRaw, fsrPercent, state);

  bool ok = mqttClient.publish(topic, payload, (unsigned int)len);
  Serial.print("PUB telemetry -> "); Serial.print(topic); Serial.print("  ");
  Serial.println(payload);
  if (!ok) Serial.println("Falha no publish telemetry");
}

void publishEvent(const char* eventName, const char* severity) {
  char topic[128];
  char payload[256];
  snprintf(topic, sizeof(topic), "chairposture/%s/event", DEVICE_ID);
  unsigned long ts = millis();
  int len = snprintf(payload, sizeof(payload),
    "{\"device_id\":\"%s\",\"ts\":%lu,\"event\":\"%s\",\"severity\":\"%s\"}",
    DEVICE_ID, ts, eventName, severity);
  bool ok = mqttClient.publish(topic, payload, (unsigned int)len);
  Serial.print("PUB event -> "); Serial.print(topic); Serial.print("  ");
  Serial.println(payload);
  if (!ok) Serial.println("Falha no publish event");
}

// ----- Calibração rápida -----
void calibrateBaseline() {
  Serial.println("Calibrando baseline: sente-se ereto e permaneça 5s...");
  const int SAMPLES = 20;
  float sumPitch = 0.0, sumRoll = 0.0;
  int validSamples = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int16_t ax_raw, ay_raw, az_raw;
    mpu.getAcceleration(&ax_raw, &ay_raw, &az_raw);
    float ax = (float)ax_raw / ACCEL_SENSITIVITY;
    float ay = (float)ay_raw / ACCEL_SENSITIVITY;
    float az = (float)az_raw / ACCEL_SENSITIVITY;
    float pitch = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
    float roll  = atan2(ay, az) * 180.0 / PI;
    sumPitch += pitch;
    sumRoll  += roll;
    validSamples++;
    delay(250);
  }
  if (validSamples > 0) {
    pitchOffset = sumPitch / validSamples;
    rollOffset  = sumRoll / validSamples;
    calibrated = true;
    Serial.printf("Calibração OK. pitchOffset=%.2f rollOffset=%.2f\n", pitchOffset, rollOffset);
  } else {
    Serial.println("Calibração falhou (nenhuma amostra).");
    }}
