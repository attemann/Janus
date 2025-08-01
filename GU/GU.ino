//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "GU.h"
#include "src/GUTX.h"
#include "src/EventDetection.h"
#include "src/GPSInterface.h"
#include "src/Receiver.h"

bool showFix = false; // Set to true to print GNSS fix details
bool showRTCM = false; // Set to true to print RTCM messages    
bool showGngga = false; // Set to true to print GNGGA messages

// GPS UART PINS
#define UART_RX       4
#define UART_TX       5

#define GNSS_BAUD 115200
//UM980 myGNSS;
HardwareSerial SerialGNSS(2); //Use UART2 on the ESP32

#define RFM69_CS 21
#define RFM69_IRQ 1
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

#define RFM69_SCK 19
#define RFM69_MISO 20
#define RFM69_MOSI 18

#define MY_NODE_ID 2
RFM69 radio(RFM69_CS, RFM69_IRQ, true);
RTCM_Reassembler reassembler;

Slope slope;

GNSSFix lastFix;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for USB serial ready (dev/debug only)

  Serial.println ("RTKF3F GU starting...");

  Serial.println("Radio: Starting");

  if (RFM69_RST != -1) {
      pinMode(RFM69_RST, OUTPUT);
      digitalWrite(RFM69_RST, LOW);
      delay(10);
      digitalWrite(RFM69_RST, HIGH);
      delay(10);
  }

  SPI.begin(RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);  // SCK, MISO, MOSI, SS (NSS)
  Serial.printf("Radio: SCK=%d, MISO=%d, MOSI=%d, CS=%d\n",
	  RFM69_SCK, RFM69_MISO, RFM69_MOSI, RFM69_CS);

  if (!radio.initialize(RF69_868MHZ, MY_NODE_ID, NETWORK_ID)) {
      Serial.println("Radio: Init failed");
      while (1)
          ;
  }
  Serial.printf("Radio: Initialized node ID %d, network ID %d\n", MY_NODE_ID, NETWORK_ID);

  // Test register read:
  byte version = radio.readReg(0x10);  // REG_VERSION
  if (version == 0x24) {
      Serial.println("Radio: RFM69 is alive (version 0x24)");
  }
  else {
      Serial.print("Radio: RFM69 not responding. Read version: 0x");
      Serial.println(version, HEX);
  }

  radio.setHighPower();  // Only if IS_RFM69HCW is true
  radio.setFrequency(RTCM_TX_FREQ);
  radio.encrypt(NULL);  // or radio.encrypt("sampleEncryptKey");
  Serial.printf("Radio: Set to frequency %.2f MHz\n", RTCM_TX_FREQ / 1000000.0);

  // --- GNSS UART ---
  SerialGNSS.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  Serial.printf("GPS: UART initialized on RX=%d, TX=%d\n", UART_RX, UART_TX);

  int portDetected = detectUM980Port(SerialGNSS);
  if (portDetected > 0) {
      Serial.printf("GPS: Detected COM%d for UM980\n", portDetected);
  }
  else {
      Serial.println("GPS: Failed to detect UM980 port.");
  }

  Serial.println("GPS: Init ok");

  Serial.println("GPS: Setting config");
  // Send kommandoer til UM980

  String Cmd = "";

  Cmd = "freset\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 5000);

  Cmd = "config signalgroup 2\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  Cmd = "mode rover uav\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  Cmd = "config rtk timeout 60\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  Cmd = "config rtk reliability 3 1\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  Cmd = "gpgga com2 1\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  Cmd = "saveconfig\r\n";
  SerialGNSS.print(Cmd);
  monitorSerial(SerialGNSS, Cmd, 500);

  // Subsystem initializations;
  initEventDetection();

  //enable airborne detector
  //airborneDetector.begin();

  Serial.println ("RTKF3F Glider unit ready!");
  showMenu();

}

void loop() {
   readConsole();

  // --- 1. Handle incoming messages from RTCM Base ---

   if (radio.receiveDone()) {
       if (radio.DATALEN > 0 && radio.DATA[0] == 0xA0) {
           // Dette er et fragment – gi det til reassembleren
           reassembler.acceptFragment(radio.DATA, radio.DATALEN);

           if (reassembler.isComplete()) {
               const uint8_t* rtcm = reassembler.getData();
               size_t rtcmLen = reassembler.getLength();

               // Valider før du sender videre
               if (isValidRTCM(rtcm, rtcmLen)) {
                   uint16_t rtcmType = getBits(rtcm, 24, 12);
                   Serial.printf("Valid RTCM [%d] %d bytes, forwarding to GNSS\r\n", rtcmType, rtcmLen);
                   SerialGNSS.write(rtcm, rtcmLen);
               }
               else {
                   Serial.println("> Invalid RTCM CRC");
               }

               reassembler.reset();  // Klar for ny melding
           }
       }
   }

  // --- 2. Poll GNSS and check for base/safety crossings ---
  GNSSFix fix;
  if (readGNSSData(fix) && showFix) {

      Serial.printf(
          "GNSS Fix:\n"
          "  Time:       %02d:%02d:%05.2f UTC\n"
          "  Latitude:   %.6f° (%c)\n"
          "  Longitude:  %.6f° (%c)\n"
          "  Altitude:   %.1f m\n"
          "  Fix type:   %d (%s)\n"
          "  Satellites: %d\n"
          "  HDOP:       %d\n"
          "  Flags:      GPS=%d  DGPS=%d  RTK-Float=%d  RTK-Fixed=%d\n\n",
          fix.hour, fix.minute, fix.second,
          fix.lat, (fix.lat < 0 ? 'S' : 'N'),
          fix.lon, (fix.lon < 0 ? 'W' : 'E'),
          fix.alt,
          fix.fixType,
          (fix.fixType == 0 ? "Invalid" :
              fix.fixType == 1 ? "GPS" :
              fix.fixType == 2 ? "DGPS" :
              fix.fixType == 3 ? "GPS PPS" :
              fix.fixType == 4 ? "RTK Fixed" :
              fix.fixType == 5 ? "RTK Float" :
              fix.fixType == 6 ? "Dead reckoning" :
              fix.fixType == 7 ? "Manual input" :
              fix.fixType == 8 ? "Simulator" : "Other"),
          fix.numSV,
          fix.HDOP,
          fix.gpsFix, fix.diffUsed, fix.rtkFloat, fix.rtkFix
      );

    lastFix=fix;

    //EventCode event;
    //if (checkForCrossingEvent(fix, event)) {
    //  txEvent(event, fixStatus(fix));
    //}

    //airborneDetector.update(fix.relNorth, fix.relEast);
  }
  else {
	  //Serial.println("No valid GNSS fix received");
  }
  delay(5);
}
