//GU.ino

#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "GU.h"

#define APPNAME "GU 1.0"

#define THIS_NODEID NODEID_GU

// Radio
inline constexpr int8_t RFM69_MISO = 20;
inline constexpr int8_t RFM69_MOSI = 18;
inline constexpr int8_t RFM69_SCK  = 19;
inline constexpr int8_t RFM69_CSS  = 21;
inline constexpr int8_t RFM69_IRQ  = 16;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;

RadioModule radioMod(RFM69_CSS, RFM69_IRQ, true);
RadioModule::RTCM_Reassembler reassembler;

bool showFix = true; // Set to true to print GNSS fix details
bool showGngga = false; // Set to true to print GNGGA messages

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
    delay(3000);

    Serial.printf("%s starting\r\n", APPNAME);

    ////////////////////////////////////

    if (!radioMod.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODEID, NETWORK_ID, FREQUENCY_RTCM)) {
        Serial.println("Radio init failed");
        while (1);
    }
    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);
    delay(1000);
    // GNSS 
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
	Serial.println("GNSS port starting...");

    if (gnss.detectUARTPort() == 0) {
        haltUnit("Gnss port", "Failure, freeze");
        radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, ERROR_COM);
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

    if (gnss.gpsDataAvailable()) {
		//gpsBuf = gnss.readGPS();    
        const uint8_t* buf = gnss.getBuffer();
        size_t len = sizeof(buf);
		switch (buf[0]) {
            case 0: // No data received
				Serial.println("GU:No data received from GNSS");    
                break;
            case '$': // NMEA message
                Serial.println("GU:Received NMEA message:");
                    if (gnss.parseGGA(buf, len, fix)) {
                        if (showFix) gnss.showFix(fix);
                    }
                    else if (buf[0] == '$' && strstr((char*)buf, "GGA")) {
                        // It is a GGA, but empty (no fenix). Optionally print a shorter note.
                        Serial.println("GU:Empty GGA (no fix)");
                    }
                break;
            case '!': // UBX message
				Serial.println("GU:Received UBX message:");
                SerialGNSS.write(buf, len);
                SerialGNSS.flush();
                break;
            case '#': // Command response
				Serial.println("GU:Received command response:");
                break;
            case 0xB5: // UBX message
				Serial.println("GU:Received UBX message:");
                SerialGNSS.write(buf, len);
                SerialGNSS.flush();
                break;
            default:
                Serial.printf("GU:Unknown GPS message: %02X\n", buf[0]);
                break;
           }
    }

    uint8_t radioBuf[1024];
    size_t radioLen = 0;
    memset(radioBuf, 0, sizeof(radioBuf));
        
   if (radioMod.receive(radioBuf, radioLen)) {
       if (radioLen > 0) {
           switch (radioBuf[0]) {
           case MSG_RTCM:   
               Serial.println("GU:MSG_RTCM:Received Full RTCM message");
               if (gnss.isValidRTCM(radioBuf, radioLen)) {
                   uint16_t type = gnss.getRTCMType(radioBuf, radioLen);
                   Serial.printf("RTCMs inserted: type [%4d] %3u bytes\n", type, radioLen);
                   SerialGNSS.write(radioBuf, radioLen);
               }
               else Serial.println(ANSI_RED "GU:Invalid full RTCM message" ANSI_RESET);
               break;

           case MSG_RTCM_FRAGMENT:   
               reassembler.acceptFragment(radioBuf, radioLen);

               if (reassembler.isComplete()) {
                   //Serial.printf("GU:MSG_RTCM_FRAGMENT complete, %u fragments received\n", reassembler.getReceivedCount());

                   const uint8_t* rtcm = reassembler.getData();
                   size_t rtcmLen = reassembler.getLength();

                   if (gnss.isValidRTCM(rtcm, rtcmLen)) {
                       Serial.println("GU:MSG_RTCM_FRAGMENT " ANSI_GREEN "Valid" ANSI_RESET);
                       uint16_t type = gnss.getRTCMType(rtcm, rtcmLen);
                       Serial.printf("Inserting " ANSI_GREEN " RTCM[%4d]" ANSI_RESET " %3u bytes\n", type, rtcmLen);
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
			   arena.decodeArenaSettings(radioBuf);
               break;
           case MSG_REQ_POS:
               Serial.println("GU:MSG_REQ_POS: Received position request");
               //txRelPos(fix, true);
               break;
           default:
               Serial.printf("GU:default: Unknown packet: 0x%02X (%s) len=%u", radioBuf[0], getMessageName(radioBuf[0]), radioLen);
               for (uint8_t i = 0; i < radioLen; i++) {
                   Serial.printf(" %02X", radioBuf[i]);
               }
               Serial.println();
               break;
           }
       }
   }
   yield(); // Allow other tasks to run
}
