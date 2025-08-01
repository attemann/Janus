#define RXD2 4  // UM980 TX → ESP32 RXD2 (D7)
#define TXD2 5  // UM980 RX ← ESP32 TXD2 (D6)

#define UM980_BAUD 115200

HardwareSerial GPS(2);

void setup() {
  // Start USB serial
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing...");



  // Start UART to UM980
  //Serial1.begin(UM980_BAUD, SERIAL_8N1, RXD2, TXD2);
  GPS.begin(UM980_BAUD,SERIAL_8N1,RXD2,TXD2);
  delay(500);

  // Send GAGGA,1 command to enable GGA output
  // Format: $CFGMSG,GAGGA,1*hh<CR><LF>
  // CRC must be correct; here we send full command with checksum
  GPS.print("gpgga 1\r\n");
  delay(100);
  Serial.println("Sent command: gpgga 1");
}

void loop() {
  // Read and forward NMEA from UM980 to USB serial
  GPS.flush(); // clear output buffer
  while (GPS.available()) {
    char c = GPS.read();
    Serial.write(c);
  }
}