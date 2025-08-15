
#include <RFM69.h>
#include <RFM69registers.h>

//#include <RFM69Debug.h>
#include <SPI.h>

#define APPNAME "Send 1.0"

// Radio ESP32-WROOM
//inline constexpr int8_t RFM69_IRQ = 4;
//inline constexpr int8_t RFM69_CSS = 5;
//inline constexpr int8_t RFM69_SCK = 18;
//inline constexpr int8_t RFM69_MISO = 19;
//inline constexpr int8_t RFM69_MOSI = 23;
//inline constexpr int8_t RFM69_RST = -1;

// Radio XIAO
inline constexpr int8_t RFM69_MISO = 20;
inline constexpr int8_t RFM69_MOSI = 18;
inline constexpr int8_t RFM69_SCK = 19;
inline constexpr int8_t RFM69_CSS = 21;
inline constexpr int8_t RFM69_IRQ = 16;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST = -1;

void debugRFM69(RFM69& radio) {
  Serial.println(F("\n--- RFM69 Debug Dump ---"));

  // Operating mode
  byte opMode = radio.readReg(REG_OPMODE);
  Serial.printf("OpMode: 0x%02X\n", opMode);

  // Frequency (in Hz)
  uint32_t frf = ((uint32_t)radio.readReg(REG_FRFMSB) << 16) | ((uint16_t)radio.readReg(REG_FRFMID) << 8) | radio.readReg(REG_FRFLSB);
  float freqHz = (float)frf * 32000000.0 / 524288.0;  // FXOSC / 2^19
  Serial.printf("Frequency: %.1f Hz\n", freqHz);

  // Bitrate
  uint16_t bitrate = ((uint16_t)radio.readReg(REG_BITRATEMSB) << 8) | radio.readReg(REG_BITRATELSB);
  float bitrateVal = 32000000.0 / bitrate;
  Serial.printf("Bitrate: %.2f bps\n", bitrateVal);

  // FSK Deviation
  uint16_t fdev = ((uint16_t)radio.readReg(REG_FDEVMSB) << 8) | radio.readReg(REG_FDEVLSB);
  float fdevHz = (float)fdev * 32000000.0 / 524288.0;
  Serial.printf("Freq Deviation: %.1f Hz\n", fdevHz);

  // Output power
  byte paLevel = radio.readReg(REG_PALEVEL);
  Serial.printf("PA Level: 0x%02X (Power: %d dBm)\n", paLevel, paLevel & 0x1F);

  // RSSI Threshold
  byte rssiThresh = radio.readReg(REG_RSSITHRESH);
  Serial.printf("RSSI Thresh: %d dB\n", -(rssiThresh / 2));

  // Sync Config
  Serial.printf("SyncConfig: 0x%02X\n", radio.readReg(REG_SYNCCONFIG));
  Serial.printf("SyncValue1 (NetworkID): 0x%02X\n", radio.readReg(REG_SYNCVALUE1));

  // Packet Config
  Serial.printf("PacketConfig1: 0x%02X\n", radio.readReg(REG_PACKETCONFIG1));
  Serial.printf("PacketConfig2: 0x%02X\n", radio.readReg(REG_PACKETCONFIG2));

  // Node & Broadcast IDs
  Serial.printf("NodeAddr: 0x%02X\n", radio.readReg(REG_NODEADRS));
  Serial.printf("BroadcastAddr: 0x%02X\n", radio.readReg(REG_BROADCASTADRS));

  // Version (check hardware)
  Serial.printf("Version: 0x%02X\n", radio.readReg(REG_VERSION));

  Serial.println(F("--- End of Dump ---\n"));
}


#define THIS 3
#define TARGET 2

RFM69 radio(RFM69_CSS, RFM69_IRQ, true);  // true = high power module (RFM69HCW)

#define LED_BUILTIN 2

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.print(APPNAME);
  Serial.println(" - Starting");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Test LED (adjust for active-low if needed)
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CSS);

  if (RFM69_RST != -1) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
  }
  if (!radio.initialize(RF69_868MHZ, THIS)) {
    Serial.println("Radio init failed");
    while (1)
      ;
  }

  radio.setHighPower(true);  // Only if IS_RFM69HCW is true
  radio.setFrequency(868200000);
  radio.encrypt(NULL);  // or radio.encrypt("sampleEncryptKey");
  radio.setAddress(THIS);
  //radio.setNetwork(100);
  //radio.writeReg(REG_SYNCVALUE1, 100);
  delay(100);
  debugRFM69(radio);

  Serial.print(APPNAME);
  Serial.println(" - RFM69 initialized");
}

void loop() {
  const char msg[] = "Test 1,2,3";
  Serial.printf("Sending from %d to %d '", THIS, TARGET);
  Serial.print(msg);
  Serial.println("'");

  radio.setMode(RF69_MODE_RX);

  debugRFM69(radio);
  radio.send(TARGET, msg, strlen(msg), false);

  delay(3000);  // Wait 3 seconds
}
