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

// ---------------- BUFFER CIRCULAR ----------------
#define BUFFER_MAX 200

struct Sample {
  uint32_t ts;      // usado internamente, mas NÃO enviado
  int16_t tensao;
  int16_t ax;
};

Sample buffer[BUFFER_MAX];
volatile int bufCount = 0;
volatile int bufWriteIndex = 0;

portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------- TASK HANDLES ----------------
TaskHandle_t taskLeitura = nullptr;
TaskHandle_t taskEnvio   = nullptr;

// ---------------- FLAGS ----------------
volatile bool enviando = false;
portMUX_TYPE muxFlag = portMUX_INITIALIZER_UNLOCKED;

// ---------------- CONTROLE DE ARMAZENAMENTO ----------------
unsigned long tempoForcaZero = 0;
bool armazenando = true;

// ---------------- FUNÇÕES WIFI/MQTT ----------------

void connectWiFi() {
  Serial.println("[WIFI] Conectando...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\n[WIFI] Conectado!");
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

void publishData(const Sample &s, uint32_t msgID) {
  char msg[200];
  snprintf(msg, sizeof(msg),
           "{\"msg_id\":%u,\"tensao\":%d,\"ax\":%d}",
           msgID, s.tensao, s.ax);

  Serial.print("[MQTT] Enviando: ");
  Serial.println(msg);

  client.publish(mqtt_topic, msg);
}

// ---------------- TASK DE ENVIO (CORE 1) ----------------
void taskEnvioMQTT(void *param) {

  Serial.println("[CORE 1] Iniciando task de envio...");

  connectWiFi();
  connectMQTT();

  static uint32_t msgID = 0;

  for (;;) {

    Serial.println("[CORE 1] Dormindo (esperando notificação)...");
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("[CORE 1] Acordei! Vou enviar os dados.");

    portENTER_CRITICAL(&muxFlag);
    enviando = true;
    portEXIT_CRITICAL(&muxFlag);

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

    for (int i = 0; i < count; i++) {
      msgID++;
      publishData(localBuf[i], msgID);
      client.loop();
      delay(5);
    }

    Serial.println("[CORE 1] Envio concluído.");

    portENTER_CRITICAL(&muxFlag);
    enviando = false;
    portEXIT_CRITICAL(&muxFlag);
  }
}

// ---------------- TASK DE LEITURA (CORE 0) ----------------
void taskLeituraSensores(void *param) {

  Serial.println("[CORE 0] Iniciando task de leitura...");

  for (;;) {

    bool enviandoLocal;
    portENTER_CRITICAL(&muxFlag);
    enviandoLocal = enviando;
    portEXIT_CRITICAL(&muxFlag);

    if (enviandoLocal) {
      Serial.println("[CORE 0] Core 1 enviando → pausa leitura");
      vTaskDelay(10);
      continue;
    }

    int16_t tensao_i = analogRead(FORCE_PIN);

    if (tensao_i <= 100) {
      Serial.printf("[CORE 0] Tensão baixa (%d) → ignorando leitura\n", tensao_i);

      if (tempoForcaZero == 0) tempoForcaZero = millis();
      if (millis() - tempoForcaZero >= 1000) armazenando = false;

      vTaskDelay(10);
      continue;
    }

    if (!armazenando) {
      Serial.println("[CORE 0] Tensão voltou > 100 → retomando armazenamento");
    }

    tempoForcaZero = 0;
    armazenando = true;

    sensors_event_t a, g, dummy;
    mpu.getEvent(&a, &g, &dummy);

    int16_t ax_i = (int16_t)(a.acceleration.x * 100);
    uint32_t ts  = millis();

    Serial.printf("[CORE 0] Leitura válida → Tensão=%d  AX=%d\n", tensao_i, ax_i);

    portENTER_CRITICAL(&bufferMux);
    buffer[bufWriteIndex] = {ts, tensao_i, ax_i};
    bufWriteIndex = (bufWriteIndex + 1) % BUFFER_MAX;
    if (bufCount < BUFFER_MAX) bufCount++;
    int currentCount = bufCount;
    portEXIT_CRITICAL(&bufferMux);

    Serial.printf("[CORE 0] Gravado no buffer (%d/%d)\n", currentCount, BUFFER_MAX);

    if (currentCount >= BUFFER_MAX) {
      Serial.println("[CORE 0] Buffer cheio → acordando Core 1");
      xTaskNotifyGive(taskEnvio);
    }

    vTaskDelay(10);
  }
}

// ---------------- TASK DE RESET AUTOMÁTICO ----------------
void taskResetAutomatico(void *param) {
  const unsigned long UMA_HORA = 3600000;
  unsigned long inicio = millis();

  for (;;) {
    if (millis() - inicio >= UMA_HORA) {
      Serial.println("[RESET] 1 hora atingida → Reiniciando ESP32...");
      delay(200);
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
    while (1) delay(1000);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  connectWiFi();
  connectMQTT();

  xTaskCreatePinnedToCore(taskLeituraSensores, "Leitura", 12000, NULL, 1, &taskLeitura, 0);
  xTaskCreatePinnedToCore(taskEnvioMQTT, "Envio", 16000, NULL, 3, &taskEnvio, 1);
  xTaskCreatePinnedToCore(taskResetAutomatico, "ResetAutomatico", 4096, NULL, 1, NULL, 1);

  Serial.println("===== SETUP CONCLUÍDO =====");
}

void loop() {
  client.loop();
  delay(10);
}