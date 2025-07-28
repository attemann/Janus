
#include <RFM69.h>
#include <SPI.h>

#define NODE_ID        1      // Unique ID for this node
#define NETWORK_ID     100    // Must match receiver
//#define GATEWAY_ID     2      // Destination node
#define FREQUENCY      RF69_868MHZ
#define RTCM_TX_FREQ 868100000

// Pin assignments for ESP32-S3
#define RFM69_CS       D1
#define RFM69_IRQ      D2
#define RFM69_IRQN     digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST      -1

  int PIN_SCK  = D8;
  int PIN_MISO = D9;
  int PIN_MOSI = D10;

RFM69 radio(RFM69_CS, RFM69_IRQ, true); // true = high power module (RFM69HCW)

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting RFM69 transmitter...");

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);

  if (RFM69_RST != -1) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
  }

  // Initialize radio
  radio.initialize(FREQUENCY, NODE_ID, NETWORK_ID);
  radio.setHighPower();         // Required for RFM69HCW
  radio.encrypt(NULL);          // No encryption
  Serial.println("RFM69 initialized");
}

void loop() {
  const char msg[] = "Hei 1,2,3";
  Serial.print("Sending: ");
  Serial.println(msg);

  radio.send(255, msg, strlen(msg));

  delay(3000); // Wait 3 seconds
}
