#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- PINOS ----------------
#define FORCE_PIN     34

// ---------------- WIFI ----------------
const char* ssid     = "f2g";
const char* password = "Ft01610220";

// ---------------- MQTT ----------------
const char* mqtt_server = "168.138.140.255";
const int   mqtt_port   = 1883;
const char* mqtt_user   = "force";
const char* mqtt_pass   = "610220";
const char* mqtt_topic  = "force_data";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------------- SENSORES ----------------
Adafruit_MPU6050 mpu;

// ---------------- CONFIG ----------------
const int SAMPLE_INTERVAL_MS = 100;    
const int DELTA_DESCARTE     = 10;     // 10% para descartar
const int DELTA_WAKE         = 10;     // 10% para acordar
const unsigned long LIMITE_DESCARTE = 15000; // 15 segundos

// ---------------- BUFFER CIRCULAR ----------------
#define BUFFER_MAX 200

struct Sample {
  uint32_t ts;
  int16_t force;
  int16_t az;
  int16_t temp;
};

Sample buffer[BUFFER_MAX];
volatile int bufCount = 0;
volatile int bufWriteIndex = 0;

portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- TASK HANDLES ----------------
TaskHandle_t taskLeitura = nullptr;
TaskHandle_t taskEnvio   = nullptr;

// ---------------- ESTADOS ----------------
volatile bool enviando = false;
portMUX_TYPE muxFlag = portMUX_INITIALIZER_UNLOCKED;

bool modoAtivo = true;                    // true = coletando normalmente
unsigned long tempoDescartando = 0;       // tempo acumulado descartando

// ---------------- VARIÁVEIS DE CONTROLE ----------------
int16_t lastAz = 0;
bool firstRead = true;

// ---------------- FUNÇÕES WIFI/MQTT ----------------

void wifiOn() {
  Serial.println("[WIFI] Ligando WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\n[WIFI] Conectado!");
}

void wifiOff() {
  Serial.println("[WIFI] Desligando WiFi...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("[WIFI] WiFi OFF");
}

void connectMQTT() {
  Serial.println("[MQTT] Conectando...");
  client.setServer(mqtt_server, mqtt_port);

  while (!client.connected()) {
    if (client.connect("ESP32-Coletor", mqtt_user, mqtt_pass)) {
      Serial.println("[MQTT] Conectado!");
    } else {
      Serial.print("[MQTT] Falhou, rc=");
      Serial.println(client.state());
      delay(300);
    }
  }
}

void publishData(const Sample &s, uint32_t id) {
  char msg[200];
  snprintf(msg, sizeof(msg),
           "{\"id\":%u,\"force\":%d,\"az\":%d,\"temp\":%d,\"ts\":%u}",
           id, s.force, s.az, s.temp, s.ts);

  Serial.print("[MQTT] Enviando: ");
  Serial.println(msg);

  client.publish(mqtt_topic, msg);
}

// ---------------- TASK DE ENVIO (CORE 1) ----------------
void taskEnvioMQTT(void *param) {

  Serial.println("[CORE 1] Task de envio iniciada");

  for (;;) {

    Serial.println("[CORE 1] Dormindo (esperando notificação)...");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("[CORE 1] Acordei! Vou enviar os dados.");

    portENTER_CRITICAL(&muxFlag);
    enviando = true;
    portEXIT_CRITICAL(&muxFlag);

    wifiOn();
    connectMQTT();

    Sample localBuf[BUFFER_MAX];
    int count;

    portENTER_CRITICAL(&bufferMux);
    count = bufCount;
    Serial.printf("[CORE 1] Copiando %d amostras do buffer\n", count);

    for (int i = 0; i < count; i++) {
      int index = (bufWriteIndex - count + i + BUFFER_MAX) % BUFFER_MAX;
      localBuf[i] = buffer[index];
    }
    bufCount = 0;
    portEXIT_CRITICAL(&bufferMux);

    uint32_t id = (uint32_t)ESP.getEfuseMac();

    for (int i = 0; i < count; i++) {
      publishData(localBuf[i], id);
      client.loop();
      delay(10);
    }

    Serial.println("[CORE 1] Envio concluído. Limpando buffer e desligando WiFi.");

    wifiOff();

    portENTER_CRITICAL(&muxFlag);
    enviando = false;
    portEXIT_CRITICAL(&muxFlag);

    Serial.println("[CORE 1] Voltando a dormir...");
  }
}

// ---------------- TASK DE LEITURA (CORE 0) ----------------
void taskLeituraSensores(void *param) {

  Serial.println("[CORE 0] Task de leitura iniciada");

  for (;;) {

    // ---------------- BLOQUEIO SE CORE 1 ESTÁ ENVIANDO ----------------
    bool enviandoLocal;
    portENTER_CRITICAL(&muxFlag);
    enviandoLocal = enviando;
    portEXIT_CRITICAL(&muxFlag);

    if (enviandoLocal) {
      Serial.println("[CORE 0] Core 1 enviando → pausa leitura");
      vTaskDelay(20);
      continue;
    }

    // ---------------- MODO DE BAIXO CONSUMO ----------------
    if (!modoAtivo) {

      Serial.println("[CORE 0] Modo de baixo consumo: dormindo 200ms");

      esp_sleep_enable_timer_wakeup(200000); // 200ms
      esp_light_sleep_start();

      sensors_event_t a2, g2, t2;
      mpu.getEvent(&a2, &g2, &t2);

      int16_t force2 = analogRead(FORCE_PIN);
      int16_t az2    = (int16_t)(a2.acceleration.z * 100);
      int16_t delta2 = abs(az2 - lastAz);
      lastAz = az2;

      Serial.printf("[CORE 0] LightSleep Wake → Força=%d Delta=%d\n", force2, delta2);

      if (force2 > 0 && delta2 > DELTA_WAKE) {
        Serial.println("[CORE 0] Movimento detectado → voltando ao modo ativo");
        modoAtivo = true;
        tempoDescartando = 0;
      }

      continue;
    }

    // ---------------- MODO ATIVO — LEITURA NORMAL ----------------
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    int16_t force_i = analogRead(FORCE_PIN);
    int16_t az_i    = (int16_t)(a.acceleration.z * 100);
    int16_t temp_i  = (int16_t)(t.temperature * 100);
    uint32_t ts     = millis();

    if (firstRead) {
      lastAz = az_i;
      firstRead = false;
    }

    int16_t delta = abs(az_i - lastAz);
    lastAz = az_i;

    Serial.printf("[CORE 0] AZ=%d  Temp=%d  Força=%d  Delta=%d\n",
                  az_i, temp_i, force_i, delta);

    // ---------------- DESCARTE DE LEITURAS ----------------
    if (force_i == 0 && delta < DELTA_DESCARTE) {

      if (tempoDescartando == 0) tempoDescartando = millis();

      unsigned long decorrido = millis() - tempoDescartando;
      Serial.printf("[CORE 0] Descartando leitura há %lu ms\n", decorrido);

      if (decorrido >= LIMITE_DESCARTE) {
        Serial.println("[CORE 0] 15s descartando → entrando em modo de baixo consumo");
        modoAtivo = false;
      }

      vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
      continue;
    }
    else {
      tempoDescartando = 0;
    }

    // ---------------- GRAVAÇÃO NO BUFFER ----------------
    portENTER_CRITICAL(&bufferMux);
    buffer[bufWriteIndex] = {ts, force_i, az_i, temp_i};
    bufWriteIndex = (bufWriteIndex + 1) % BUFFER_MAX;
    if (bufCount < BUFFER_MAX) bufCount++;
    int currentCount = bufCount;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("[CORE 0] Gravado no buffer (%d/%d)\n", currentCount, BUFFER_MAX);

    if (currentCount >= BUFFER_MAX) {
      Serial.println("[CORE 0] Buffer cheio → acordando Core 1");
      xTaskNotifyGive(taskEnvio);
    }

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("===== INICIANDO SISTEMA =====");

  if (!mpu.begin()) {
    Serial.println("ERRO: MPU6050 não encontrado!");
    while (1) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  wifiOff();

  client.setServer(mqtt_server, mqtt_port);

  xTaskCreatePinnedToCore(taskLeituraSensores, "Leitura", 12000, NULL, 1, &taskLeitura, 0);
  xTaskCreatePinnedToCore(taskEnvioMQTT, "Envio", 16000, NULL, 3, &taskEnvio, 1);

  Serial.println("===== SETUP CONCLUÍDO =====");
}

void loop() {
  delay(20);
}