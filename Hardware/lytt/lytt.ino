#include <RFM69.h>
#include <SPI.h>

#define NODE_ID        2        // This is the Gateway ID (receiver)
#define NETWORK_ID     100
#define FREQUENCY      RF69_868MHZ

#define RFM69_CS       10
#define RFM69_IRQ      9
#define RFM69_RST      -1

#define LED_BUILTIN 2

RFM69 radio(RFM69_CS, RFM69_IRQ, true);

void setup() {
  Serial.begin(115200);
  delay(10);

  SPI.begin( 12, 13, 11, 10); // Explicit SPI pins, matching sender

  if (RFM69_RST != -1) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
  }

  Serial.println("Starting RFM69 receiver...");

  if (!radio.initialize(FREQUENCY, NODE_ID, NETWORK_ID)) {
    Serial.println("Radio initialization failed!");
    while (1);
  }

  radio.setHighPower();
  radio.encrypt(NULL); // No encryption, matching sender

  Serial.println("RFM69 initialized");
}

void loop() {
  if (radio.receiveDone()) { // Check if data is received
    if (radio.DATALEN > 0) {
      Serial.print("Received from Node ");
      Serial.print(radio.SENDERID);
      Serial.print(": [");
      for (byte i = 0; i < radio.DATALEN; i++) {
        Serial.print((char)radio.DATA[i]);
      }
      Serial.print("], RSSI: ");
      Serial.println(radio.RSSI);

      // Acknowledge receipt (optional, but useful for reliability)
      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }

      // Blink LED to indicate reception
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}