// code for XIAO ESP32C6
#include <RFM69.h>
#include <SPI.h>

#define NODE_ID 2  // This is the Gateway ID (receiver)
#define NETWORK_ID 100
#define FREQUENCY RF69_868MHZ
#define RTCM_TX_FREQ 868100000

#define ESP32C6       1
#define ESP32WROOM32  2
#define HOST ESP32C6  // <-- Velg en

#if HOST == ESP32C6
  #define RFM69_CS 21
  #define RFM69_IRQ 1
  #define RFM69_SCK 19
  #define RFM69_MISO 20
  #define RFM69_MOSI 18
#elif HOST == ESP32WROOM32
  #define RFM69_CS 5
  #define RFM69_IRQ 4
  #define RFM69_SCK 18
  #define RFM69_MISO 19
  #define RFM69_MOSI 23
#else
  #error "Unknown HOST defined"
#endif

  #define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
  #define RFM69_RST -1

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

  Serial.printf("MISO %d\n", RFM69_MISO);
  Serial.printf("MOSI %d\n", RFM69_MOSI);
  Serial.printf("SCK  %d\n", RFM69_SCK);
  Serial.printf("CS   %d\n", RFM69_CS);
  Serial.printf("IRQ  %d\n", RFM69_IRQ);

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);

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
    while (1)
      ;
  }

  radio.setHighPower();
  radio.setFrequency(RTCM_TX_FREQ);  // Default receive frequency for RTC
  radio.encrypt(NULL);               // No encryption, matching sender

  if (verifyRadio(radio)) {
    Serial.println("> Radio verified");
  } else {
    Serial.println("> Radio verification failed, check wiring and settings. Freeze");
    while (1)
      ;  // Lock system if radio verification fails
  }

  Serial.println("RFM69 initialized");
}

void loop() {
  if (radio.receiveDone()) {
    if (radio.DATALEN > 0) {
      if (radio.TARGETID == NODE_ID) {
        Serial.printf("To ME from Node %d: [", radio.SENDERID);
      } else {
        Serial.printf("To %d from Node %d: [", radio.TARGETID, radio.SENDERID);
      }
      for (byte i = 0; i < radio.DATALEN; i++) {
        Serial.print((char)radio.DATA[i]);
      }
      Serial.printf("], RSSI: %d\n", radio.RSSI);

      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }
    }
  }

  delay(10);  // Mindre delay for å ikke miste meldinger
}