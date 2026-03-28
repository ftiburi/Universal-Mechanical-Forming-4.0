/*
 Coletor BLE (ESP32)
 - Lê força no pino analógico 34
 - Lê aceleração (X/Y/Z), giroscópio (X/Y/Z) e temperatura do MPU6050
 - Monta payload de 20 bytes
 - Envia via BLE advertising continuamente (sem deep sleep)
*/

#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
const int sensorPin = 34;

void setup() {
  Serial.begin(115200);
  delay(10);

  // Inicializa ADC
  analogReadResolution(12);
  analogSetPinAttenuation(sensorPin, ADC_11db);

  // Inicializa MPU6050
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("Falha ao inicializar MPU6050!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Inicializa BLE
  BLEDevice::init("ESP32-Coletor");
}

void loop() {
  // Leitura dos sensores
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  int rawForce = analogRead(sensorPin);

  // Escala valores
  uint32_t ts = millis() / 1000;
  int16_t force_i = (int16_t)rawForce;
  int16_t ax_i = (int16_t)(a.acceleration.x * 100);
  int16_t ay_i = (int16_t)(a.acceleration.y * 100);
  int16_t az_i = (int16_t)(a.acceleration.z * 100);
  int16_t gx_i = (int16_t)(g.gyro.x * 100);
  int16_t gy_i = (int16_t)(g.gyro.y * 100);
  int16_t gz_i = (int16_t)(g.gyro.z * 100);
  int16_t temp_i = (int16_t)(temp.temperature * 100);

  // Imprime no Serial
  Serial.println("=== Dados coletados ===");
  Serial.printf("Timestamp: %u\n", ts);
  Serial.printf("Força: %d\n", force_i);
  Serial.printf("Accel X: %d, Y: %d, Z: %d\n", ax_i, ay_i, az_i);
  Serial.printf("Gyro X: %d, Y: %d, Z: %d\n", gx_i, gy_i, gz_i);
  Serial.printf("Temp: %d\n", temp_i);
  Serial.println("=======================");

  // Monta payload (20 bytes)
  uint8_t payload[20];
  payload[0]  = ts & 0xFF;
  payload[1]  = (ts >> 8) & 0xFF;
  payload[2]  = (ts >> 16) & 0xFF;
  payload[3]  = (ts >> 24) & 0xFF;
  payload[4]  = force_i & 0xFF;
  payload[5]  = (force_i >> 8) & 0xFF;
  payload[6]  = ax_i & 0xFF;
  payload[7]  = (ax_i >> 8) & 0xFF;
  payload[8]  = ay_i & 0xFF;
  payload[9]  = (ay_i >> 8) & 0xFF;
  payload[10] = az_i & 0xFF;
  payload[11] = (az_i >> 8) & 0xFF;
  payload[12] = gx_i & 0xFF;
  payload[13] = (gx_i >> 8) & 0xFF;
  payload[14] = gy_i & 0xFF;
  payload[15] = (gy_i >> 8) & 0xFF;
  payload[16] = gz_i & 0xFF;
  payload[17] = (gz_i >> 8) & 0xFF;
  payload[18] = temp_i & 0xFF;
  payload[19] = (temp_i >> 8) & 0xFF;

  // Converte para String
  String mdata;
  for (int i = 0; i < 20; i++) mdata += (char)payload[i];

  // Envia via BLE advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  BLEAdvertisementData advData;
  advData.setManufacturerData(mdata);
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();
  delay(200); // tempo mínimo para envio
  pAdvertising->stop();

  Serial.println("Payload enviado via BLE.");

  delay(20); // intervalo entre leituras/envios
}