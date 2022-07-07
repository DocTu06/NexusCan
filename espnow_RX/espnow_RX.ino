#include <esp_now.h>
#include <WiFi.h>
 
// Define a data structure
typedef struct struct_message {
  float temp;
  float hum;
  float prs;
  int air;
} struct_message;
 
// Create a structured object
struct_message myData;
 
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print(myData.temp);
  Serial.print(",");
  Serial.print(myData.prs);
  Serial.print(",");
  Serial.print(myData.hum);
  Serial.print(",");
  Serial.print(myData.air);
  Serial.println();
}
 
void setup() {
  // Set up Serial Monitor
  Serial.begin(9600);
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
 
}
