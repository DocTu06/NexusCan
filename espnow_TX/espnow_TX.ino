#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <BH1750.h>
#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x24, 0xD7, 0xEB, 0x11, 0xC6, 0x9C};
//library declatrations
typedef struct struct_message {
  float temp;
  float hum;
  float prs;
  int air;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

#define SEALEVELPRESSURE_HPA (1013.25)//for altitude calculation
int i = 0;
char c;
bool newdata = false;
//LoRa module pins
int RXPin = 16;
int TXPin = 17;
//GPS pins
int GPSBaud = 9600;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
//mpu6050 variables
TinyGPSPlus gps;
//initialize gps object
SoftwareSerial gpsSerial(RXPin, TXPin);
//gps module declarations
Adafruit_BME280 bme;
//temperature, humidity and pressure sensor declaration
BH1750 lightMeter;
//light sensor
unsigned long previousMillis = 0;
const long interval = 500;
//variables for Lora delay
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  //disabling brownout protection because ealrier models of the esp32 gave false brownout errors
  Wire.begin(); //begin i2c communication
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  bme.begin(0x76); //initialize bme280
  pinMode(5, OUTPUT);

  //set the cs pins of the SPI modules as outputs to modify them(the libraries can't do that)
  Serial.begin(9600); //begin serial comunication
  if (!SD.begin(5)) {
    Serial.println("Card Mount Failed");
  }
  //SD check
  gpsSerial.begin(GPSBaud); //GPS initialization
  lightMeter.begin();
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      unsigned long currentMillis = millis();
      digitalWrite(5, LOW);
      //select the SD card module
      File file = SD.open("/nexus.txt", FILE_APPEND); //open the file
      file.print(bme.readTemperature());
      file.print(",");
      file.print(bme.readPressure() / 100.0F);
      file.print(",");
      file.print(bme.readHumidity());
      file.print(",");
      //bme280 data
      file.print(gps.location.lat(), 6);
      file.print(",");
      file.print(gps.location.lng(), 6);
      file.print(",");
      file.print(gps.time.hour());
      file.print(":");
      file.print(gps.time.minute());
      file.print(":");
      file.print(gps.time.second());
      file.print(".");
      file.print(gps.time.centisecond());
      file.print(",");
      //gps data
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_addr, 14, true);
      AcX = Wire.read() << 8 | Wire.read();
      AcY = Wire.read() << 8 | Wire.read();
      AcZ = Wire.read() << 8 | Wire.read();
      Tmp = Wire.read() << 8 | Wire.read();
      GyX = Wire.read() << 8 | Wire.read();
      GyY = Wire.read() << 8 | Wire.read();
      GyZ = Wire.read() << 8 | Wire.read();
      file.print(AcX);
      file.print(",");
      file.print(AcY);
      file.print(",");
      file.print(AcZ);
      file.print(",");
      file.print(GyX);
      file.print(",");
      file.print(GyY);
      file.print(",");
      file.print(GyZ);
      file.print(",");
      float lux = lightMeter.readLightLevel();
      file.print(lux);
      file.print(",");
      file.print(analogRead(A0));
      file.print("t");
      //accelerometer and gyroscope data
      file.close(); //close file
      if (currentMillis - previousMillis >= 500)
      {
      previousMillis = currentMillis;
      myData.temp = bme.readTemperature();
      myData.prs = (bme.readPressure() / 100.0F);
      myData.hum = bme.readHumidity();
      int air = analogRead(A0);
      myData.air = air;
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      if (result == ESP_OK) {
        Serial.println("Sending confirmed");
      }
      else {
        Serial.println("Sending error");
      }
    }
    }
  }
}
