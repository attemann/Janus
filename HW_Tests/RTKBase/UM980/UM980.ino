#include <Arduino.h>
#include <HardwareSerial.h>

// Adjust to your wiring:
#define GNSS_RX  4   // UM980 RX <- ESP32 TX
#define GNSS_TX 5   // UM980 TX -> ESP32 RX
#define GNSS_BAUD 115200

HardwareSerial SerialGNSS(2);

void setup() {
  Serial.begin(115200);               // USB serial for PC
  while(!Serial);
  delay(1000);

  Serial.println("UART passthrough test to UM980...");

  SerialGNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  while (!SerialGNSS);
  while (SerialGNSS.available()) SerialGNSS.read();  // flush garbage

  SerialGNSS.print("versiona\r\n");
//  SerialGNSS.print("gpgga com2 1\r\n");
//  SerialGNSS.print("versiona com3\r\n");

}

void loop() {
  // Forward data from UM980 to PC
  while (SerialGNSS.available()) {
    Serial.write(SerialGNSS.read());
  }

  // Forward data from PC to UM980
  while (Serial.available()) {
    SerialGNSS.write(Serial.read());
  }
}
