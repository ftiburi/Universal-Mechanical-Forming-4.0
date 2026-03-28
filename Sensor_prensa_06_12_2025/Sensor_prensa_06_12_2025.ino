/****************************************************
 * Projeto ESP32 + MPU6050 + MQTT
 * Autor: Fabio
 * 
 * Funcionalidades:
 * - Leitura de acelerômetro e giroscópio (MPU6050)
 * - Leitura de força via potenciômetro (sensor analógico)
 * - Conexão Wi-Fi
 * - Envio dos dados via MQTT com buffer circular
 ****************************************************/

#include <FS.h>
#include <FSImpl.h>
#include <vfs_api.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ---------- Configuração do MPU6050 ----------
Adafruit_MPU6050 mpu;
const int sensorPin = 34;   // pino analógico para leitura de força
float sensorValue = 0;

// ---------- Configuração Wi-Fi ----------
const char* ssid       = "f2g";
const char* password   = "Ft01610220";

// ---------- Configuração MQTT ----------
const char* mqtt_server   = "168.138.140.255";
const int   mqtt_port     = 1883;
const char* mqtt_topic    = "force_data";
const char* mqtt_user     = "force";
const char* mqtt_password = "610220";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- Buffer Circular ----------
#define BUFFER_SIZE 256
int buffer[BUFFER_SIZE];
int head = 0;  // início
int tail = 0;  // fim

// ---------- Variáveis globais ----------
int vatual = 0;
int vultimo = 0;

// ---------- Funções de Buffer ----------
void addToBuffer(int data) {
  int next = (tail + 1) % BUFFER_SIZE;
  if (next == head) {
    Serial.println("Buffer cheio! Dados descartados.");
    return;
  }
  buffer[tail] = data;
  tail = next;
}

int removeFromBuffer() {
  if (head == tail) {
    Serial.println("Buffer vazio!");
    return -1;
  }
  int data = buffer[head];
  head = (head + 1) % BUFFER_SIZE;
  return data;
}

bool isBufferEmpty() {
  return head == tail;
}

// ---------- Conexão Wi-Fi ----------
void cntwf() {
  Serial.print("Conectando à rede Wi-Fi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.print(".");
  }
  Serial.println("Wi-Fi conectado!");
  Serial.print("Endereço IP: ");
  Serial.println(WiFi.localIP());
}

// ---------- Conexão MQTT ----------
void cntMQTT() {
  while (!client.connected()) {
    Serial.print("Tentando conexão MQTT...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("Conectado ao MQTT.");
    } else {
      Serial.print("Falha na conexão MQTT, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5 segundos...");
      delay(5000);
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Inicializa Wi-Fi
  cntwf();

  // Configura cliente MQTT
  client.setServer(mqtt_server, mqtt_port);

  // Inicializa MPU6050
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar MPU6050!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 encontrado!");

  // Configura faixas de operação
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Configuração concluída.");
}

// ---------- Loop Principal ----------
void loop() {
  // Leitura do potenciômetro (força simulada)
  sensorValue = analogRead(sensorPin);
  float force = sensorValue;

  // Leitura do MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Impressão Serial
  Serial.print("Acceleration X: "); Serial.print(a.acceleration.x);
  Serial.print(", Y: "); Serial.print(a.acceleration.y);
  Serial.print(", Z: "); Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: "); Serial.print(g.gyro.x);
  Serial.print(", Y: "); Serial.print(g.gyro.y);
  Serial.print(", Z: "); Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: "); Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Force: "); Serial.print(force / 100);
  Serial.println(" N");

  // Adiciona valor ao buffer se houver mudança significativa
  vatual = force;
  if (abs(vatual - vultimo) > 100) {
    vultimo = vatual;
    addToBuffer(vatual);
  }

  // Envia dados via MQTT
  if (!isBufferEmpty()) {
    cntMQTT();
    int valueToSend = removeFromBuffer();
    if (valueToSend != -1) {
      char msg[50];
      snprintf(msg, 50, "%d", valueToSend);
      client.publish(mqtt_topic, msg);
      Serial.print("Valor enviado via MQTT: ");
      Serial.println(valueToSend);
    }
  }

  Serial.println();
  delay(500);
}
