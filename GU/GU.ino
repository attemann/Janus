//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "GU.h"
#include "src/GUTX.h"
#include "src/EventDetection.h"
#include "src/GPSInterface.h"
#include "src/Receiver.h"

#define APPNAME "GU 1.0"

// Radio
#define RFM69_CS 21
#define RFM69_IRQ 1
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

#define RFM69_SCK 19
#define RFM69_MISO 20
#define RFM69_MOSI 18

RadioModule::HWPins radioPins = {
    .sck   = RFM69_SCK,
    .miso  = RFM69_MISO,
    .mosi  = RFM69_MOSI,
    .cs    = RFM69_CS,
    .irq   = RFM69_IRQ,
    .reset = RFM69_RST
};

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);
RadioModule::RTCM_Reassembler reassembler;

bool showFix = false; // Set to true to print GNSS fix details
bool showRTCM = false; // Set to true to print RTCM messages    
bool showGngga = false; // Set to true to print GNGGA messages

// UART
#define GNSS_BAUD 115200
#define UART_RX       4
#define UART_TX       5

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);
GNSSModule::GNSSFix fix;

void haltUnit(String msg1, String msg2) {
    Serial.print(msg1);
    Serial.print(":");
    Serial.println(msg2);
    while (true);
}

Slope slope;

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);
    Serial.printf("%s booting\r\n", APPNAME);

    // Radio
    if (!radioMod.init(radioPins, NODEID_GU, NETWORK_ID, RTCM_TX_FREQ)) {
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        haltUnit("Radio verify", "Failure, freeze");
    }
    else Serial.println("Radio verified");

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);

    if (gnss.detectUARTPort() == 0) {
        haltUnit("Gnss port", "Failure, freeze");
    }
    else Serial.println("Gnss port ok");

    gnss.sendCommand("unlog\r\n");
    //gnss.sendReset();
    gnss.sendCommand("config signalgroup 2\r\n");
    gnss.sendCommand("mode rover uav\r\n");
    gnss.sendCommand("config rtk timeout 60\r\n");
    //gnss.sendCommand("config rtk reliability 3 1\r\n");
    gnss.sendCommand("gpgga com2 1\r\n");
    gnss.sendCommand("saveconfig\r\n");

  Serial.println ("RTKF3F Glider unit ready!");
  showMenu();

}

void loop() {
   readConsole();

   uint8_t* data = nullptr;
   uint8_t len = 0;

   // 1. Check for received radio packet
   if (radioMod.receive(data, len)) {
       if (len > 0) {
           switch (data[0]) {
           case MSG_RTCM:  // Full RTCM message (sent in one radio packet)
               if (gnss.isValidRTCM(data, len)) {
                   SerialGNSS.write(data, len);
                   Serial.printf("Full RTCM %u bytes sent to GNSS\n", len);
               }
               else {
                   Serial.println("Invalid full RTCM message");
               }
               break;
           case MSG_RTCMFRAGMENT:  // Fragmented RTCM
               reassembler.acceptFragment(data, len);
               if (reassembler.isComplete()) {
                   const uint8_t* rtcm = reassembler.getData();
                   size_t rtcmLen = reassembler.getLength();

                   if (gnss.isValidRTCM(rtcm, rtcmLen)) {
                       SerialGNSS.write(rtcm, rtcmLen);
                   }
                   reassembler.reset();
               }
               break;
           case MSG_FLIGHT_SETTINGS:
               Serial.println("Received Flight settings");
               if (len >= 9) {
                   uint16_t angle10 = data[1] | (data[2] << 8);
                   bool aBaseLeft = data[3];
                   uint8_t gliderId = data[4];

                   int16_t offsetN_cm = data[5] | (data[6] << 8);
                   int16_t offsetE_cm = data[7] | (data[8] << 8);

                   slope.setGliderId(gliderId);
                   slope.setSlopeAngle(angle10 / 10.0f);
                   slope.setABaseLeft(aBaseLeft);
                   slope.setPilotOffsetNED(offsetN_cm / 100.0f, offsetE_cm / 100.0f, 0);

                   initEventDetection();
               }
               break;

           case MSG_REQ_POS: {
               // send relative pos to base (true), not pilot (false)
               Serial.println("Received position request");
               txRelPos(fix, true);
               break;
           }
           default:
               Serial.printf("Unknown radio packet type: 0x%02X\n", data[0]);
               break;
           }
       }
   }

  if (gnss.readGNSSData(fix, showGngga)) {
	  if (showFix) gnss.showFix(fix);
  }
  
  delay(5);
}
