#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------------- PINOS ----------------
#define FORCE_PIN 34

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

// ---------------- MPU ----------------
Adafruit_MPU6050 mpu;

// ---------------- FILA MQTT ----------------
struct Sample {
  int16_t tensao;
  int16_t ax;
  uint32_t msgID;
};

QueueHandle_t filaMQTT;

// ---------------- WIFI ----------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  Serial.println("[WIFI] Conectando...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
  Serial.println("\n[WIFI] Conectado!");
}

// ---------------- MQTT ----------------
void connectMQTT() {
  Serial.println("[MQTT] Conectando ao broker...");
  while (!client.connected()) {
    if (client.connect("ESP32-Coletor", mqtt_user, mqtt_pass)) {
      Serial.println("[MQTT] Conectado!");
    } else {
      Serial.print("[MQTT] Falhou, rc=");
      Serial.println(client.state());
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
}

// ---------------- TASK MQTT LOOP (CORE 1) ----------------
void taskMQTTLoop(void *param) {
  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      connectWiFi();
    }
    if (!client.connected()) {
      connectMQTT();
    }
    client.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---------------- TASK ENVIO MQTT (CORE 1) ----------------
void taskEnvio(void *param) {
  Sample s;

  for (;;) {
    if (xQueueReceive(filaMQTT, &s, portMAX_DELAY) == pdTRUE) {
      char msg[150];
      snprintf(msg, sizeof(msg),
               "{\"msg_id\":%u,\"tensao\":%d,\"ax\":%d}",
               s.msgID, s.tensao, s.ax);

      Serial.print("[MQTT] Enviando: ");
      Serial.println(msg);

      if (!client.connected()) {
        connectMQTT();
      }
      client.publish(mqtt_topic, msg);
    }
  }
}

// ---------------- TASK LEITURA SENSORES (CORE 0) ----------------
void taskLeitura(void *param) {
  uint32_t msgID = 0;

  for (;;) {
    int16_t tensao = analogRead(FORCE_PIN);

    if (tensao > 100) {
      sensors_event_t a, g, dummy;
      mpu.getEvent(&a, &g, &dummy);

      Sample s;
      s.tensao = tensao;
      s.ax = (int16_t)(a.acceleration.x * 100);
      s.msgID = ++msgID;

      xQueueSend(filaMQTT, &s, 0);

      Serial.printf("[LEITURA] Tensão=%d  AX=%d  msgID=%u\n",
                    s.tensao, s.ax, s.msgID);
    } else {
      Serial.printf("[LEITURA] Tensão baixa (%d) → ignorando\n", tensao);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ---------------- TASK RESET AUTOMÁTICO (OPCIONAL) ----------------
void taskResetAutomatico(void *param) {
  const unsigned long UMA_HORA = 3600000;
  unsigned long inicio = millis();

  for (;;) {
    if (millis() - inicio >= UMA_HORA) {
      Serial.println("[RESET] 1 hora atingida → Reiniciando ESP32...");
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP.restart();
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("===== INICIANDO SISTEMA =====");

  if (!mpu.begin()) {
    Serial.println("ERRO: MPU6050 não encontrado!");
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);

  filaMQTT = xQueueCreate(300, sizeof(Sample));
  if (filaMQTT == NULL) {
    Serial.println("ERRO: não foi possível criar a fila MQTT!");
    while (1) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  xTaskCreatePinnedToCore(taskMQTTLoop, "MQTTLoop", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(taskEnvio,    "Envio",    4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskLeitura,  "Leitura",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskResetAutomatico, "ResetAutomatico", 4096, NULL, 1, NULL, 1);

  Serial.println("===== SETUP CONCLUÍDO =====");
}

// ---------------- LOOP VAZIO ----------------
void loop() {
  // Tudo é feito nas tasks FreeRTOS
}