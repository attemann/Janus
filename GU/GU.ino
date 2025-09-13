// GU.ino
#include <Arduino.h>
#include <HardwareSerial.h>
//#include <SPI.h>
#include <RFM69.h>
#include <RTKF3F.h>
#include "GU.h"
#include "Glider.h"
#include "Arena.h"

#define APPNAME "GU 1.0"
#define THIS_NODE_ID NODEID_GU
#define COMMANDDELAY 1000  // ms to wait for GNSS command response

// Radio
inline constexpr int8_t RFM69_MISO = 20;
inline constexpr int8_t RFM69_MOSI = 18;
inline constexpr int8_t RFM69_SCK  = 19;
inline constexpr int8_t RFM69_CSS  = 21;
inline constexpr int8_t RFM69_IRQ  = 16;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;



bool showFix = true;
bool showGngga = false;

// UART (GNSS)
#define GNSS_BAUD 115200
#define UART_RX   5
#define UART_TX   4

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);
GNSSFix fix;

RadioModule radio(RFM69_CSS, RFM69_IRQ, true);
RadioModule::RTCM_Reassembler reassembler;

Arena  arena;
Glider glider;

// static unsigned long lastSpeakMs = 0;
#define SPEAK_INTERVAL_MS 30000

static inline void haltUnit(const String& msg1, const String& msg2) {
    Serial.print(msg1); Serial.print(": "); Serial.println(msg2);
    while (true);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(200);
    Serial.printf("%s starting\n", APPNAME);

    if (!radio.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        while (true) delay(100);
    }

    radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
        static_cast<uint8_t>(DEVICE_STARTING));

    Serial.println("Radio started");

    // GNSS
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
	gnss.setRadio(&radio);  
    
    delay(500);
    if (!gnss.sendWait("unlog", "response: OK", COMMANDDELAY)) {
        //haltUnit("GNSS", "No response");
    }

	bool gnssConnected = false;
    const char* ports[] = { "com1", "com2", "com3" };
    char cmd[32];

    for (int i = 0; i < 3; ++i) {
        snprintf(cmd, sizeof(cmd), "versiona %s", ports[i]);
        Serial.printf("Trying %s...\n", ports[i]);

        if (gnss.sendWait(cmd, "#VERSIONA", COMMANDDELAY)) {
            Serial.printf("✔️  GNSS replied on %s\n", ports[i]);
            radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, (uint8_t)ERR_GPS);
            gnssConnected = true;
            break;
        }
        delay(200);
    }

    if (!gnssConnected) {
        Serial.println("❌ No GNSS COM port responded.");
        //haltUnit("GNSS", "no reply");
    }

    // Configure rover + NMEA
    if (!gnss.sendWait("unlog"                , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config signalgroup 2" , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config pvtalg multi"  , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config rtk timeout 60", "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("mode rover"           , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("gpgga com2 1"         , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("saveconfig"           , "response: OK", COMMANDDELAY)) { 

        radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);
        Serial.println("RTKF3F Glider unit ready!");
        showMenu();
	} else haltUnit("GNSS", "Config failed");
}

void loop() {
    readConsole();      // your menu/console

    // Use GNSSModule's pumpGGA function instead of pumpGnssOnce
    if (gnss.pumpGGA(fix)) {
        if (showFix) {
            Serial.printf("GGA OK: Time=%02d:%02d:%.2f FIX=%u SIV=%d HDOP=%.2f lat=%.6f lon=%.6f elev=%.2f\n",
                fix.hour, fix.minute, fix.second, (unsigned)fix.type, fix.SIV, fix.HDOP, fix.lat, fix.lon, fix.elevation);
        }

        // Send periodic SIV message to base station
        static unsigned long lastSpeakMs = 0;
        if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
            radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
            lastSpeakMs = millis();
        }
    }

    // Handle incoming radio packets directly (no queue system)
    uint8_t buffer[255];
    size_t len = sizeof(buffer);

    if (radio.receive(buffer, len) && len > 0) {
        switch (buffer[0]) {
        case MSG_RTCM: {
            if (len <= 1) break;

            const uint8_t* rtcm = buffer + 1;     // Skip MSG_RTCM byte
            size_t rlen = len - 1;

            if (rlen >= 6 && rtcm[0] == 0xD3) {
                uint16_t type = gnss.getRTCMType(rtcm, rlen);
                Serial.printf("RTCM received: type [%4u] %u bytes\n", type, (unsigned)rlen);
                SerialGNSS.write(rtcm, rlen);       // Forward to GNSS UART
            }
            break;
        }

        case MSG_RTCM_FRAGMENT: {
            if (len <= 4) break;

            reassembler.acceptFragment(buffer + 4, len - 4);  // Skip header

            if (reassembler.isComplete()) {
                const uint8_t* rtcm = reassembler.getData();
                size_t rlen = reassembler.getLength();

                if (rlen >= 6 && rtcm[0] == 0xD3 && gnss.isValidRTCM(rtcm, rlen)) {
                    uint16_t type = gnss.getRTCMType(rtcm, rlen);
                    Serial.printf("RTCM reassembled: type [%4u] %u bytes\n", type, (unsigned)rlen);
                    SerialGNSS.write(rtcm, rlen);
                }
                reassembler.reset();
            }
            break;
        }

        case MSG_ARENA_SETTINGS: {
            Serial.println("Arena settings received");
            arena.decodeArenaSettings(buffer);
            break;
        }

        case MSG_REQ_POS: {
            Serial.println("Position request received");
            // Could send fix data back to base station
            break;
        }

        default: {
            Serial.printf("Unknown packet: 0x%02X, len=%u\n", buffer[0], len);
            break;
        }
        }
    }

    yield();
}