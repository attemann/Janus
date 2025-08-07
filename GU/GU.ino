//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>

#include "GU.h"


#define APPNAME "GU 1.0"

// Radio
#define RFM69_CS 21
#define RFM69_IRQ 1
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

#define RFM69_SCK 19
#define RFM69_MISO 20
#define RFM69_MOSI 18

const char* getMessageName(uint8_t id) {
    switch (id) {
    case MSG_RTCM_NUM_SENT:    return "RTCM_NUMSENT";
    case MSG_RTCM:             return "RTCM";
    case MSG_RTCM_FRAGMENT:    return "RTCMFRAGMENT";
    case MSG_ARENA_SETTINGS:   return "FLIGHT_SETTINGS";
	case MSG_GLIDER_SETTINGS:  return "GLIDER_SETTINGS";
    case MSG_REQ_POS:          return "REQ_POS";
    default:                   return "UNKNOWN";
    }
}

RadioModule::HWPins radioPins = {
    .sck   = RFM69_SCK,
    .miso  = RFM69_MISO,
    .mosi  = RFM69_MOSI,
    .cs    = RFM69_CS,
    .irq   = RFM69_IRQ,
    .reset = RFM69_RST,
	.irqn = RFM69_IRQN
};

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);
RadioModule::RTCM_Reassembler reassembler;

bool showFix = true; // Set to true to print GNSS fix details
bool showGngga = false; // Set to true to print GNGGA messages
bool showRTCM = false; // Set to true to print GNGGA messages

// UART
#define GNSS_BAUD 115200
#define UART_RX       4
#define UART_TX       5

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);
GNSSModule::GNSSFix fix;

Arena arena;
Glider glider;

void haltUnit(String msg1, String msg2) {
    Serial.print(msg1);
    Serial.print(":");
    Serial.println(msg2);
    while (true);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(500);

    Serial.printf("%s starting\r\n", APPNAME);
 
    // Radio
    if (!radioMod.init(radioPins, NODEID_GU, NETWORK_ID, GU_TX_FREQ)) {
        radioMod.sendMessageCode(NODEID_CD, GU_TX_FREQ, RTCM_TX_FREQ, MSG_ERROR, ERROR_RADIO_INIT);
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        radioMod.sendMessageCode(NODEID_CD, GU_TX_FREQ, RTCM_TX_FREQ, MSG_ERROR, ERROR_RADIO_VERIFY);
        haltUnit("Radio verify", "Failure, freeze");
    }
    else Serial.println("Radio verified");

    radioMod.sendMessageCode(NODEID_CD, GU_TX_FREQ, RTCM_TX_FREQ, MSG_INFORMATION, INFO_DEVICE_STARTING);

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
	Serial.println("GNSS port starting...");

    if (gnss.detectUARTPort() == 0) {
        haltUnit("Gnss port", "Failure, freeze");
        radioMod.sendMessageCode(0, GU_TX_FREQ, RTCM_TX_FREQ, MSG_ERROR, ERROR_COM);
    }
    else Serial.println("Gnss port ok");

    gnss.sendCommand("unlog\r\n");
    gnss.sendCommand("mode rover\r\n");
    gnss.sendCommand("config rtk timeout 60\r\n");
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
           case MSG_RTCM:   // Full RTCM message (sent in one radio packet)
               Serial.println("GU:MSG_RTCM:Received Full RTCM message");
               if (gnss.isValidRTCM(data, len)) {
                   uint16_t type = gnss.getRTCMType(data,len);
                   Serial.printf("RTCMs inserted: type [%4d] %3u bytes\n", type, len);
                   SerialGNSS.write(data, len);
               }
               else Serial.println(ANSI_RED "GU:Invalid full RTCM message" ANSI_RESET);
               break;

           case MSG_RTCM_FRAGMENT:   // Fragmented RTCM
			   //Serial.println("GU:MSG_RTCM_FRAGMENT:Received fragment");
               reassembler.acceptFragment(data, len);

			   //radioMod.printRTCMSegment(data, len);
               if (reassembler.isComplete()) {
                   //Serial.printf("GU:MSG_RTCM_FRAGMENT complete, %u fragments received\n", reassembler.getReceivedCount());

                   const uint8_t* rtcm = reassembler.getData();
                   size_t rtcmLen = reassembler.getLength();

                   //for (size_t i = 0; i < rtcmLen; ++i) {
                   //    Serial.printf("%02X ", rtcm[i]);
                   //    if ((i + 1) % 16 == 0) Serial.println();
                   //}
                   //Serial.println();

                   // Strong debug: show length, expected from header
                   //uint16_t headerPayloadLen = ((rtcm[1] & 0x3F) << 8) | rtcm[2];
                   //Serial.printf("  Buffer length: %u\n", (unsigned)rtcmLen);
                   //Serial.printf("  RTCM header payload length: %u\n", headerPayloadLen);
                   //Serial.printf("  Total RTCM expected length (header+payload+CRC): %u\n", (unsigned)(3 + headerPayloadLen + 3));
                   //Serial.printf("  First byte: 0x%02X\n", rtcm[0]);

                   // Now do validity check
                   if (gnss.isValidRTCM(rtcm, rtcmLen)) {
                       Serial.println("GU:MSG_RTCM_FRAGMENT " ANSI_GREEN "Valid" ANSI_RESET);
                       uint16_t type = gnss.getRTCMType(rtcm, rtcmLen);
                       Serial.printf("Inserting " ANSI_GREEN " RTCM[%4d]" ANSI_RESET " %3u bytes\n", type, len);
                       SerialGNSS.write(rtcm, rtcmLen);
                   }
                   else {
                       Serial.println("GU:MSG_RTCM_FRAGMENT " ANSI_RED "Invalid" ANSI_RESET);
                   }
                   reassembler.reset();
               }
               break;
           case MSG_ARENA_SETTINGS:
               Serial.println("GU:MSG_ARENA_SETTINGS: Received Arena settings");
			   arena.decodeArenaSettings(data);
               break;
           case MSG_REQ_POS:
               // send relative pos to base (true), not pilot (false)
               Serial.println("GU:MSG_REQ_POS: Received position request");
               //txRelPos(fix, true);
               break;
           default:
               Serial.printf("GU:default: Unknown packet: 0x%02X (%s) len=%u", data[0], getMessageName(data[0]), len);
               for (uint8_t i = 0; i < len; i++) {
                   Serial.printf(" %02X", data[i]);
               }
               Serial.println();
               break;
           }
       }
   }

  if (gnss.readGNSSData(fix, showGngga)) {
	  if (showFix) gnss.showFix(fix);
      //Serial.println("---");
  }
  
  delay(2);
}
