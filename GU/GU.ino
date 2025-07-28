//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "src/GUTX.h"
#include "src/EventDetection.h"
#include "src/GPSInterface.h"
#include "src/Receiver.h"
//#include "src/Airborne.h"
#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS

// GPS UART PINS
#define GNSS_BAUD 115200
#define UART_TX       17
#define UART_RX       16

//UM980 myGNSS;
UM980 myGNSS;
HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

#define MY_NODE_ID 2
RFM69 rxtxRadio;
Slope slope;

GNSSFix lastFix;

//AirborneDetector airborneDetector;
int detectUM980Port(HardwareSerial& gnssSerial) {
    const char* testCommands[] = {
        "versiona com1",
        "versiona com2",
        "versiona com3"
    };

    for (int port = 1; port <= 3; port++) {
        gnssSerial.flush(); // Clear any previous data
        delay(100);

        Serial.printf("Testing COM%d...\n", port);
        gnssSerial.print(testCommands[port - 1]);
        gnssSerial.print("\r\n");

        unsigned long startTime = millis();
        String response = "";

        while (millis() - startTime < 500) {
            while (gnssSerial.available()) {
                char c = gnssSerial.read();
                response += c;
            }
        }

        if (response.indexOf("VERSIONA") != -1 || response.indexOf("#VERSIONA") != -1) {
            Serial.printf("✅ UM980 responded on COM%d\n", port);
            return port;
        }
    }

    Serial.println("❌ No UM980 COM port responded.");
    return -1; // None found
}

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for USB serial ready (dev/debug only)

  Serial.println ("RTKF3F GU starting...");

  // --- GNSS UART ---
  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  int portDetected = detectUM980Port(SerialGNSS);
  if (portDetected > 0) {
      Serial.printf("Detected COM%d for UM980\n", portDetected);
  }
  else {
      Serial.println("Failed to detect UM980 port.");
  }

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
      Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
      while (true);
  }
  //myGNSS.enableDebugging(); // Print all debug to Serial

  Serial.println("> GPS init ok");



  Serial.println("Setting GPS config");
  SerialGNSS.println("unlog");
  SerialGNSS.println("mode rover");
  SerialGNSS.println("gpgga com2 1");
  SerialGNSS.println("saveconfig");

  // Radio setup
  rxtxRadio.initialize(RF69_868MHZ, MY_NODE_ID, NETWORK_ID);
  rxtxRadio.setHighPower();               // Needed for RFM69HCW
  rxtxRadio.setFrequency(RTCM_TX_FREQ);   // Default receive frequency for RTCM
  rxtxRadio.setPowerLevel(31);            // Max power level (adjust as needed)

  // Subsystem initializations;
  initEventDetection();

  //enable airborne detector
  //airborneDetector.begin();

  // Optional: visual feedback
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);

  Serial.println ("RTKF3F Glider unit ready!");
}

void loop() {
  // --- 1. Handle incoming messages from RTCM Base ---
  if (rxtxRadio.receiveDone()) {
	  Serial.println("Received radio message");
    handleRadioMessage(rxtxRadio.DATA, rxtxRadio.DATALEN);  // e.g., RTCM or MSG_FLIGHTSETTINGS
  }

  // --- 2. Poll GNSS and check for base/safety crossings ---
  GNSSFix fix;

  if (readGNSSFix(fix)) {

    Serial.printf("GNSS Fix: %.2f N, %.2f E\n", fix.relNorth, fix.relEast);
    Serial.println("");

    // store last fix
    lastFix=fix;

    EventCode event;

    if (checkForCrossingEvent(fix, event)) {
      txEvent(event, fixStatus(fix));
    }

    //airborneDetector.update(fix.relNorth, fix.relEast);
  }
  else {
	  Serial.println("No valid GNSS fix received");
  }

  // Optional: blink LED on valid GNSS fix
  // digitalWrite(LED_BUILTIN, fix.gpsFix ? HIGH : LOW);
  delay(200);
}
