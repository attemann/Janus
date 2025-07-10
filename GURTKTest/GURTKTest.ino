// XIAO ESP32-C6: Send "GAGGA,1" to UM980 and read NMEA output

#define RXD2 17  // UM980 TX → ESP32 RXD2 (D7)
#define TXD2 16  // UM980 RX ← ESP32 TXD2 (D6)

#define UM980_BAUD 115200

void setup() {
  // Start USB serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing...");

  // Start UART to UM980
  Serial1.begin(UM980_BAUD, SERIAL_8N1, RXD2, TXD2);
  delay(500);

  // Send GAGGA,1 command to enable GGA output
  // Format: $CFGMSG,GAGGA,1*hh<CR><LF>
  // CRC must be correct; here we send full command with checksum
  Serial1.print("mode rover\r\n");
  Serial.println("Sent command: mode rover");
  Serial1.print("gpgga 1\r\n");
  Serial.println("Sent command: gpgga 1");
}

void loop() {
  // Read and forward NMEA from UM980 to USB serial
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);
  }
}