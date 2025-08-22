//GU.ino

#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include <RadioTask.h>
#include "GU.h"

#define APPNAME "GU 1.0"

#define THIS_NODE_ID NODEID_GU

// Radio
inline constexpr int8_t RFM69_MISO = 20;
inline constexpr int8_t RFM69_MOSI = 18;
inline constexpr int8_t RFM69_SCK  = 19;
inline constexpr int8_t RFM69_CSS  = 21;
inline constexpr int8_t RFM69_IRQ  = 16;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;

//RadioModule radioMod(RFM69_CSS, RFM69_IRQ, true);
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
    if (!radioStartTask(
        RFM69_MISO, RFM69_MOSI, RFM69_SCK, RFM69_CSS, RFM69_IRQ,
        THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
	    Serial.println(APPNAME " Radio task start failed");
        while (true);  // halt
    }

    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, static_cast <uint8_t>(DeviceState::DEVICE_STARTING));
    delay(1000);
    // GNSS 
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
    Serial.println(APPNAME "GNSS starting...");

    if (gnss.detectUARTPort() == 0) {
        radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, static_cast <uint8_t>(DeviceState::DEVICE_STARTING));
        Serial.println("GNSS init failed");
        while (1);
    }
    else Serial.println("GNSS port ok");

    gnss.sendCommand("unlog\r\n");
    gnss.sendCommand("mode rover\r\n");
    gnss.sendCommand("config rtk timeout 60\r\n");
    gnss.sendCommand("gpgga com2 1\r\n");
    gnss.sendCommand("saveconfig\r\n");    
    
    radioSendMsg(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);

  Serial.println ("RTKF3F Glider unit ready!");
  showMenu();
}

void loop() {
    readConsole();
    
    RxPacket pkt;

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
        
   if (radioReceive(pkt, 0)) {
       if (pkt.len > 0) {
           switch (pkt.data[0]) {
           case MSG_RTCM:   
               Serial.println("GU:MSG_RTCM:Received Full RTCM message");
               if (gnss.isValidRTCM(pkt.data, pkt.len)) {
                   uint16_t type = gnss.getRTCMType(pkt.data, pkt.len);
                   Serial.printf("RTCMs inserted: type [%4d] %3u bytes\n", type, pkt.len);
                   SerialGNSS.write(pkt.data, pkt.len);
               }
               else Serial.println(ANSI_RED "GU:Invalid full RTCM message" ANSI_RESET);
               break;

           case MSG_RTCM_FRAGMENT:   
               reassembler.acceptFragment(pkt.data, pkt.len);

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
           case MSG_ARENA_SETTINGS:
               Serial.println("GU:MSG_ARENA_SETTINGS: Received Arena settings");
			   arena.decodeArenaSettings(pkt.data);
               break;
           case MSG_REQ_POS:
               Serial.println("GU:MSG_REQ_POS: Received position request");
               //txRelPos(fix, true);
               break;
           default:
               Serial.printf("GU:default: Unknown packet: 0x%02X\r\n", pkt.data[0]);
               for (uint8_t i = 0; i < pkt.len; i++) {
                   Serial.printf(" %02X", pkt.data[i]);
               }
               Serial.println();
               break;
           }
       }
   }
   yield(); // Allow other tasks to run
}
