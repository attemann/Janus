#include <RFM69.h>
#include <SPI.h>

#define NODE_ID        1        // This is the Gateway ID (receiver)
#define NETWORK_ID     100
#define FREQUENCY      RF69_868MHZ

//#define RFM69_CS       10
//#define RFM69_IRQ      9
//#define RFM69_RST      -1

#define RFM69_CS       5
#define RFM69_IRQ      4
#define RFM69_IRQN     digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST      -1

  int PIN_SCK  = 18;
  int PIN_MISO = 19;
  int PIN_MOSI = 23;


#define LED_BUILTIN 2

RFM69 radio(5, 4, true);

void setup() {

    pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // Test LED (adjust for active-low if needed)
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(10);

  //SPI.begin( 12, 13, 11, 10); // Explicit SPI pins, matching sender
    // Start SPI with custom pins
 // SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);

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