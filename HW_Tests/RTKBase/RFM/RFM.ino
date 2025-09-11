#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>

// --- Your wiring ---
inline constexpr int8_t  RFM69_IRQ  = 8;   // DIO0 -> ESP32
inline constexpr int8_t  RFM69_CSS  = 10;  // NSS  -> ESP32
inline constexpr int8_t  RFM69_SCK  = 12;
inline constexpr int8_t  RFM69_MISO = 13;
inline constexpr int8_t  RFM69_MOSI = 11;
inline constexpr int8_t  RFM69_RST  = -1;  // no reset line wired

// --- Radio setup ---
#define NETWORKID   100
#define NODEID      1
#define TONODEID    2              // change if you have a second node; else it's fine
#define FREQUENCY   RF69_868MHZ    // 868 MHz for EU
#define IS_HIGHPOWER true          // HCW/HW variants need high-power mode


// Construct with custom pins + SPI instance

RFM69 radio(RFM69_CSS, RFM69_IRQ, IS_HIGHPOWER, &SPI);


static void pulseResetIfWired() {
  if (RFM69_RST >= 0) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(1);
    digitalWrite(RFM69_RST, HIGH);
    delay(5);
    digitalWrite(RFM69_RST, LOW);
    delay(5);
  }
}

void setup() {
  Serial.begin(115200);

  // Init SPI on your chosen pins
  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CSS);

  if (!radio.initialize(FREQUENCY, NODEID, NETWORKID)) {
    Serial.println(F("Radio init failed!"));
  } else {
    Serial.println(F("Radio init OK"));
  }
  radio.setHighPower(true);
  radio.setPowerLevel(20);
}

uint32_t lastTx = 0;

void loop() {
  // Receive (non-blocking). If something arrives, print it.
  if (radio.receiveDone()) {
    Serial.printf("RX from %d, len=%d, RSSI=%d: ", radio.SENDERID, radio.DATALEN, radio.RSSI);
    for (uint8_t i = 0; i < radio.DATALEN; i++) {
      char c = (radio.DATA[i] >= 32 && radio.DATA[i] <= 126) ? char(radio.DATA[i]) : '.';
      Serial.print(c);
    }
    Serial.println();

    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println(F("ACK sent."));
    }
  }

  // Transmit a short beacon every ~2 seconds
  if (millis() - lastTx > 2000) {
    lastTx = millis();
    char msg[40];
    snprintf(msg, sizeof(msg), "Hello from node %d @%lu", NODEID, (unsigned long)millis());
    bool ok = radio.sendWithRetry(TONODEID, msg, strlen(msg), 2 /*retries*/, 40 /*ackWait ms*/);
    Serial.printf("TX -> node %d: \"%s\"  %s\r\n", TONODEID, msg, ok ? "OK (ACK)" : "no ACK");
  }

  // Print a quick RSSI heartbeat now and then
  static uint32_t lastRssi = 0;
  if (millis() - lastRssi > 3000) {
    lastRssi = millis();
    Serial.printf("Live RSSI: %d dBm\r\n", radio.readRSSI());
  }
}