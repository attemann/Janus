
#include <RFM69.h>
#include <SPI.h>

#define NODE_ID        1      // Unique ID for this node
#define NETWORK_ID     100    // Must match receiver
#define GATEWAY_ID     2      // Destination node
#define FREQUENCY      RF69_868MHZ

// Pin assignments for ESP32-S3
//#define RFM69_CS       10     // NSS
//#define RFM69_IRQ      9      // DIO0
#define RFM69_CS       1     // NSS
#define RFM69_IRQ      2      // DIO0
#define RFM69_IRQN     digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST      -1     // Not used here

  int PIN_SCK  = 19;
  int PIN_MISO = 20;
  int PIN_MOSI = 18;

RFM69 radio(RFM69_CS, RFM69_IRQ, true); // true = high power module (RFM69HCW)

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting RFM69 transmitter...");

  // Define your SPI pins


  // Start SPI with custom pins
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);

  // Initialize radio
  radio.initialize(FREQUENCY, NODE_ID, NETWORK_ID);
  radio.setFrequency(868300000);
  radio.setHighPower();         // Required for RFM69HCW
  radio.encrypt(NULL);          // No encryption
  Serial.println("RFM69 initialized");
}

void loop() {
  const char msg[] = "Hei 1,2,3";
  Serial.print("Sending: ");
  Serial.println(msg);

  radio.send(GATEWAY_ID, msg, strlen(msg));

  delay(100); // Wait 3 seconds
}
