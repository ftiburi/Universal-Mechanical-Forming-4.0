#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- WIFI ----------------
const char* ssid = "f2g";
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
const int FORCE_PIN = 34;

// ---------------- CONFIG ----------------
const int SAMPLE_INTERVAL_MS   = 1000;   // leitura a cada 1 s
const int WAKE_THRESHOLD_I16   = 147;    // 15% de 9.81 m/s² * 100
const int SEND_INTERVAL_MS     = 10000;  // envia MQTT pelo menos a cada 10 s

// Wake-up timer (1 segundo)
const uint64_t WAKEUP_INTERVAL_US = 1000000ULL;

// ---------------- BUFFER ----------------
#define BUFFER_MAX 200
struct Sample {
  uint32_t ts;
  int16_t force;
  int16_t az;
  int16_t temp;
};
Sample buffer[BUFFER_MAX];
volatile int bufCount = 0;

portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- TASK HANDLES ----------------
TaskHandle_t taskLeitura;
TaskHandle_t taskEnvio;

// ---------------- CONTROLE ENVIO PERIÓDICO ----------------
unsigned long lastSendMillis = 0;

// ---------------- FUNÇÕES ----------------

void connectWiFi() {
  Serial.print("[WIFI] Conectando a ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.print("\n[WIFI] Conectado, IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("[MQTT] Conectando ao broker...");
    if (client.connect("ESP32-Coletor", mqtt_user, mqtt_pass)) {
      Serial.println("conectado!");
    } else {
      Serial.print("falhou, rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

void publishData(const Sample &s, uint32_t id) {
  char msg[200];
  snprintf(msg, sizeof(msg),
    "{\"id\":%u,\"force\":%d,\"az\":%d,\"temp\":%d,\"ts\":%u}",
    id, s.force, s.az, s.temp, s.ts
  );

  bool ok = client.publish(mqtt_topic, msg);
  Serial.printf("[MQTT] Enviando: %s  -> %s\n", msg, ok ? "OK" : "FALHA");
}

// ---------------- TASK DE ENVIO (CORE 0) ----------------
void taskEnvioMQTT(void *param) {
  Serial.println("[CORE 0] Task de envio INICIALIZADA");

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("[CORE 0] Acordei para enviar MQTT");

    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    connectMQTT();

    portENTER_CRITICAL(&bufferMux);
    int count = bufCount;
    portEXIT_CRITICAL(&bufferMux);

    uint32_t id = (uint32_t)ESP.getEfuseMac();

    portENTER_CRITICAL(&bufferMux);
    Sample localBuf[BUFFER_MAX];
    for (int i = 0; i < count; i++) {
      localBuf[i] = buffer[i];
    }
    bufCount = 0;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("[CORE 0] Enviando %d amostras ao broker...\n", count);

    for (int i = 0; i < count; i++) {
      publishData(localBuf[i], id);
      client.loop();
      delay(20);
    }

    Serial.println("[CORE 0] Envio concluído. Dormindo até ser acordado...");
    vTaskSuspend(NULL);
  }
}

// ---------------- TASK DE LEITURA (CORE 1) ----------------
void taskLeituraSensores(void *param) {
  Serial.println("[CORE 1] Task de leitura INICIALIZADA");

  for (;;) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    int16_t force_i = analogRead(FORCE_PIN);
    int16_t az_i    = (int16_t)(a.acceleration.z * 100);
    int16_t temp_i  = (int16_t)(t.temperature * 100);
    uint32_t ts     = millis() / 1000;

    Serial.printf("[CORE 1] Força=%d  AZ=%d  Temp=%d  ts=%u\n",
                  force_i, az_i, temp_i, ts);

    // Armazena no buffer
    portENTER_CRITICAL(&bufferMux);
    if (bufCount < BUFFER_MAX) {
      buffer[bufCount++] = {ts, force_i, az_i, temp_i};
    }
    int currentCount = bufCount;
    portEXIT_CRITICAL(&bufferMux);

    // Se buffer cheio → acorda Core 0
    if (currentCount >= BUFFER_MAX) {
      Serial.println("[CORE 1] Buffer cheio → notificando Core 0");
      xTaskNotifyGive(taskEnvio);
    }

    // Deep sleep SOMENTE quando buffer está vazio E sem movimento
    if (currentCount == 0 && abs(az_i) < WAKE_THRESHOLD_I16) {
      Serial.println("[CORE 1] Sem movimento e buffer vazio → deep sleep");
      esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL_US);
      esp_deep_sleep_start();
    }

    vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n===== INÍCIO DO SISTEMA =====");

  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("[MPU] ERRO: MPU6050 não encontrado!");
    while (1) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();

  lastSendMillis = millis();

  xTaskCreatePinnedToCore(taskEnvioMQTT, "Envio", 8000, NULL, 1, &taskEnvio, 0);
  xTaskCreatePinnedToCore(taskLeituraSensores, "Leitura", 8000, NULL, 1, &taskLeitura, 1);

  Serial.println("===== SETUP CONCLUÍDO =====");
}

// ---------------- LOOP ----------------
void loop() {
  client.loop();
  delay(100);
}