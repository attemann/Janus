#include <RFM69.h>
#include <SPI.h>

#define NODE_ID 2  // This is the Gateway ID (receiver)
#define NETWORK_ID 100
#define FREQUENCY RF69_868MHZ
#define RTCM_TX_FREQ 868100000

#define RFM69_CS 10
#define RFM69_IRQ 9
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

int PIN_SCK = 12;
int PIN_MISO = 13;
int PIN_MOSI = 11;


#define LED_BUILTIN 2

RFM69 radio(RFM69_CS, RFM69_IRQ, true);

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Test LED (adjust for active-low if needed)
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  delay(10);
  
  // Start SPI with custom pins
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);

  if (RFM69_RST != -1) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
  }

  Serial.println("Starting RFM69 receiver...");

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, RFM69_CS);  // SCK, MISO, MOSI, SS (NSS)

  if (!radio.initialize(RF69_868MHZ, NODE_ID, NETWORK_ID)) {
    Serial.println("Radio init failed");
    while (1)
      ;
  }

    // Test register read:
  byte version = radio.readReg(0x10);  // REG_VERSION
  if (version == 0x24) {
    Serial.println("✅ RFM69 is alive (version 0x24)");
  } else {
    Serial.print("❌ RFM69 not responding. Read version: 0x");
    Serial.println(version, HEX);
  }

  //radio.writeReg(0x37, 0b00000010); // REG_PACKETCONFIG1 = 0x371
  //radio.writeReg(0x3A, 255);  // REG_BROADCASTADRS = 0xFF
  
  radio.setHighPower();  // Only if IS_RFM69HCW is true
  radio.setFrequency(RTCM_TX_FREQ);

  radio.setAddress(NODE_ID);      // Re-apply address after setFrequency
  radio.setNetwork(NETWORK_ID);   // Optional but good practice

  radio.encrypt(NULL);  // or radio.encrypt("sampleEncryptKey");
  Serial.println("RFM69 initialized");
}

void loop() {
  if (radio.receiveDone()) {
    // Accept if message is for me or broadcasted
    //Serial.println("TargetId=" + String(radio.TARGETID));
    if (radio.TARGETID == NODE_ID || radio.TARGETID == 0) {
      Serial.print("Received from Node ");
      Serial.print(radio.SENDERID);
      Serial.print(" to ");
      Serial.print(radio.TARGETID);
      Serial.print(": [");
      for (byte i = 0; i < radio.DATALEN; i++) {
        Serial.print((char)radio.DATA[i]);
      }
      Serial.print("], RSSI: ");
      Serial.println(radio.RSSI);

      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }

      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.print("Ignored packet for node ");
      Serial.println(radio.TARGETID);
    }
  }
}