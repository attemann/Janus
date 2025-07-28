// code for XIAO ESP32C6
#include <RFM69.h>
//#include <SPI.h>

#define NODE_ID        50        // This is the Gateway ID (receiver)
#define NETWORK_ID     100
#define FREQUENCY      RF69_868MHZ
#define RTCM_TX_FREQ 868100000

/*
10:22:26.097 -> D1:  1
10:22:26.097 -> D2:  2
10:22:26.097 -> D3:  21
10:22:26.097 -> D4:  22
10:22:26.097 -> D5:  23
10:22:26.097 -> D6:  16
10:22:26.097 -> D7:  17
10:22:26.097 -> D8:  19
10:22:26.097 -> D9:  20
10:22:26.097 -> D10:  18
*/

#define RFM69_CS  3     // GPIO5
#define RFM69_IRQ  1     // GPIO4 (DIO0)
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

int PIN_SCK  = 5;       // GPIO10 (D10)
int PIN_MISO = 4;        // GPIO9  (D9)
int PIN_MOSI = 6;        // GPIO8  (D8)

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

RFM69 radio(RFM69_CS, RFM69_IRQ, true);

bool verifyRadio(RFM69& radio) {
    uint8_t version = radio.readReg(REG_VERSION);
    if (version != EXPECTED_RFM69_VERSION) {
        Serial.printf("verifyRadio: Unexpected RFM69 version 0x%02X (expected 0x%02X)\n", version, EXPECTED_RFM69_VERSION);
        return false;
    }
    return true;
}

void setup() {

  Serial.begin(115200);
  delay(500);

    pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);  // Start with LED off

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);

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
  radio.setFrequency(RTCM_TX_FREQ);   // Default receive frequency for RTC
  radio.encrypt(NULL); // No encryption, matching sender

  if (verifyRadio(radio)) {
    Serial.println("> Radio verified");
}
else {
    Serial.println("> Radio verification failed, check wiring and settings. Freeze");
    while (1); // Lock system if radio verification fails
}

  Serial.println("RFM69 initialized");
}

void loop() {
        // Blink LED to indicate reception
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);

  if (radio.receiveDone()) { // Check if data is received
    Serial.println("data received");
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


    }
  }
  delay(1000);
}