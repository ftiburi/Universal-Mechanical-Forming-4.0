#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
const int FORCE_PIN = 34;

// ---------------- CONFIG ----------------
const int SAMPLE_INTERVAL_MS = 100;    // 10 Hz
const int ACC_THRESHOLD = 100;         // 10% de variação (az*100)
const uint64_t WAKEUP_INTERVAL_US = 1000000ULL; // 1 segundo

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
TaskHandle_t taskLeitura = nullptr;
TaskHandle_t taskEnvio   = nullptr;

// ---------------- VARIÁVEIS DE CONTROLE ----------------
int16_t lastAz = 0;
bool firstRead = true;

// ---------------- FUNÇÕES ----------------

void connectWiFi() {
  Serial.print("[WIFI] Conectando a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(200);
  }
  Serial.print("\n[WIFI] Conectado! IP: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT() {
  Serial.println("[MQTT] Tentando conectar...");
  while (!client.connected()) {
    if (client.connect("ESP32-Coletor", mqtt_user, mqtt_pass)) {
      Serial.println("[MQTT] Conectado!");
    } else {
      Serial.print("[MQTT] Falhou, rc=");
      Serial.println(client.state());
      delay(800);
    }
  }
}

void publishData(const Sample &s, uint32_t id) {
  char msg[200];
  snprintf(msg, sizeof(msg),
           "{\"id\":%u,\"force\":%d,\"az\":%d,\"temp\":%d,\"ts\":%u}",
           id, s.force, s.az, s.temp, s.ts);

  bool ok = client.publish(mqtt_topic, msg);
  Serial.printf("[MQTT] Enviando: %s  -> %s\n", msg, ok ? "OK" : "FALHA");
}

// ---------------- TASK DE ENVIO (CORE 0) ----------------
void taskEnvioMQTT(void *param) {
  Serial.println("[CORE 0] Task de envio INICIALIZADA");

  for (;;) {
    Serial.println("[CORE 0] Aguardando notificação...");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    Serial.println("[CORE 0] ACORDEI! Vou enviar MQTT.");

    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    connectMQTT();

    portENTER_CRITICAL(&bufferMux);
    int count = bufCount;
    Sample localBuf[BUFFER_MAX];
    for (int i = 0; i < count; i++) localBuf[i] = buffer[i];
    bufCount = 0;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("[CORE 0] Enviando %d amostras...\n", count);

    uint32_t id = (uint32_t)ESP.getEfuseMac();

    for (int i = 0; i < count; i++) {
      publishData(localBuf[i], id);
      client.loop();
      delay(10);
    }

    Serial.println("[CORE 0] Envio concluído.");
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
    uint32_t ts     = millis();

    Serial.printf("[CORE 1] Leitura → AZ=%d  Temp=%d  Força=%d\n",
                  az_i, temp_i, force_i);

    // Primeira leitura
    if (firstRead) {
      lastAz = az_i;
      firstRead = false;
      Serial.println("[CORE 1] Primeira leitura registrada.");
    }

    // Cálculo da variação
    int16_t delta = abs(az_i - lastAz);
    lastAz = az_i;

    Serial.printf("[CORE 1] Delta AZ = %d\n", delta);

    // Armazena no buffer
    portENTER_CRITICAL(&bufferMux);
    if (bufCount < BUFFER_MAX) {
      buffer[bufCount++] = {ts, force_i, az_i, temp_i};
    }
    int currentCount = bufCount;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("[CORE 1] Buffer = %d/%d\n", currentCount, BUFFER_MAX);

    // Se buffer cheio → acorda Core 0
    if (currentCount >= BUFFER_MAX) {
      Serial.println("[CORE 1] Buffer cheio → notificando Core 0");
      xTaskNotifyGive(taskEnvio);
    }

    // 🔥 LÓGICA PRINCIPAL: Dormir quando NÃO há variação > 10%
    if (delta < ACC_THRESHOLD && currentCount == 0) {
      Serial.println("[CORE 1] Sem variação significativa → entrando em deep sleep");
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
  Serial.println("[MPU] Inicializado.");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  connectMQTT();

  BaseType_t r1 = xTaskCreatePinnedToCore(
    taskEnvioMQTT, "Envio", 12000, NULL, 2, &taskEnvio, 0);

  BaseType_t r2 = xTaskCreatePinnedToCore(
    taskLeituraSensores, "Leitura", 12000, NULL, 1, &taskLeitura, 1);

  Serial.printf("[SETUP] Task envio criada:   %s\n", r1 == pdPASS ? "OK" : "FALHA");
  Serial.printf("[SETUP] Task leitura criada: %s\n", r2 == pdPASS ? "OK" : "FALHA");

  Serial.println("===== SETUP CONCLUÍDO =====");
}

// ---------------- LOOP ----------------
void loop() {
  client.loop();
  delay(20);
}