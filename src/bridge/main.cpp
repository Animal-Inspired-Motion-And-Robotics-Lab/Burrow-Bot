//   ____       _     _             
//  |  _ \     (_)   | |            
//  | |_) |_ __ _  __| | __ _  ___  
//  |  _ <| '__| |/ _` |/ _` |/ _ \ 
//  | |_) | |  | | (_| | (_| |  __/ 
//  |____/|_|  |_|\__,_|\__, |\___| 
//                       __/ |      
//                      |___/   

// Forwards every byte in both directions with minimal delay

#define BRIDGE_RX_PIN 44
#define BRIDGE_TX_PIN 43
#define BAUD_RATE 460800

#include <Arduino.h>

void setup() {
  // USB to PC
  Serial.begin(BAUD_RATE);
  
  // UART to Platform
  Serial1.begin(BAUD_RATE, SERIAL_8N1, BRIDGE_RX_PIN, BRIDGE_TX_PIN);
  
  // Small buffers are fine - we forward immediately 
  delay(500);
  
  // Optional: blink once to show ready
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // Forward PC → Platform (motor commands)
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }
  
  // Forward Platform → PC (IMU data)
  if (Serial1.available()) {
    Serial.write(Serial1.read());
  }
  
  // No delay - run as fast as possible
}
