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
        !gnss.sendWait("gpgga 1"              , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("saveconfig"           , "response: OK", COMMANDDELAY)) { 

        radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);
        Serial.println("RTKF3F Glider unit ready!");
        showMenu();
	} // else haltUnit("GNSS", "Config failed");
}



    void processGNSSData() {
        // Use GNSSModule's pumpGGA function instead of pumpGnssOnce
        if (gnss.pumpGGA(fix)) {
            if (showFix) {
                Serial.printf("GGA OK: Time=%02d:%02d:%.2f FIX=%u SIV=%d HDOP=%.2f lat=%.6f lon=%.6f elev=%.2f\n",
                    fix.hour, fix.minute, fix.second, (unsigned)fix.type, fix.SIV, fix.HDOP, fix.lat, fix.lon, fix.elevation);
            }

            // Send periodic SIV message to base station
            sendPeriodicUpdate();
        }
    }

    void sendPeriodicUpdate() {
        static unsigned long lastSpeakMs = 0;
        if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
            radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);

            lastSpeakMs = millis();
        }
    }

    void processRadioPackets() {
        uint8_t buffer[255];
        size_t len = sizeof(buffer);

        if (radio.receive(buffer, len) && len > 0) {
            // Log received packet info
            Serial.printf("RX: %u bytes from node %u (RSSI: %d)\n",
                len, radio.getSenderId(), radio.getLastRSSI());

            handleRadioMessage(buffer, len);
        }
    }

    void handleRadioMessage(const uint8_t* buffer, size_t len) {
        if (!buffer || len == 0) return;

        switch (buffer[0]) {
        case MSG_RTCM:
            handleFullRTCM(buffer, len);
            break;

        case MSG_RTCM_FRAGMENT:
            handleFragmentedRTCM(buffer, len);
            break;

        case MSG_ARENA_SETTINGS:
            handleArenaSettings(buffer, len);
            break;

        case MSG_REQ_POS:
            handlePositionRequest(buffer, len);
            break;

        case MSG_SIV:
            handleSIVMessage(buffer, len);
            break;

        default:
            handleUnknownMessage(buffer, len);
            break;
        }
    }

    void handleFullRTCM(const uint8_t* buffer, size_t len) {
        if (len <= 1) {
            Serial.println("⚠️ Invalid RTCM packet: too short");
            return;
        }

        const uint8_t* rtcm = buffer + 1;     // Skip MSG_RTCM byte
        size_t rlen = len - 1;

        if (!validateAndInjectRTCM(rtcm, rlen, "full")) {
            Serial.println("⚠️ Failed to process full RTCM packet");
        }
    }

    void handleFragmentedRTCM(const uint8_t* buffer, size_t len) {
        if (len <= 4) {
            Serial.println("⚠️ Invalid RTCM fragment: too short");
            return;
        }

        // Extract fragment info for logging
        uint8_t fragIndex = buffer[1];
        uint8_t totalFrags = buffer[2];
        uint8_t msgId = buffer[3];

        Serial.printf("RTCM fragment %u/%u (msgId=%u)\n", fragIndex + 1, totalFrags, msgId);

        // Accept the fragment (skip the 4-byte header)
        reassembler.acceptFragment(buffer, len);

        if (reassembler.isComplete()) {
            const uint8_t* rtcm = reassembler.getData();
            size_t rlen = reassembler.getLength();

            if (!validateAndInjectRTCM(rtcm, rlen, "reassembled")) {
                Serial.println("⚠️ Failed to process reassembled RTCM packet");
            }

            reassembler.reset();
        }
    }

    bool validateAndInjectRTCM(const uint8_t* rtcm, size_t rlen, const char* source) {
        // Validate RTCM format
        if (rlen < 6 || rtcm[0] != 0xD3) {
            Serial.printf("⚠️ Invalid RTCM format from %s source\n", source);
            return false;
        }

        // Additional validation using GNSS module
        if (!gnss.isValidRTCM(rtcm, rlen)) {
            Serial.printf("⚠️ RTCM validation failed from %s source\n", source);
            return false;
        }

        // Extract and log message type
        uint16_t type = gnss.getRTCMType(rtcm, rlen);
        Serial.printf("✓ RTCM %s: type [%4u] %u bytes\n", source, type, (unsigned)rlen);

        // Inject to GNSS receiver
        size_t written = SerialGNSS.write(rtcm, rlen);
        if (written != rlen) {
            Serial.printf("⚠️ RTCM injection incomplete: %u/%u bytes written\n", written, (unsigned)rlen);
            return false;
        }

        // Ensure data is transmitted
        SerialGNSS.flush();

        // Optional: Update statistics
        updateRTCMStats(type, rlen);

        return true;
    }

    void handleArenaSettings(const uint8_t* buffer, size_t len) {
        Serial.printf("Arena settings received (%u bytes)\n", len);

        //if (arena.decodeArenaSettings(buffer)) {
        //    Serial.println("✓ Arena settings applied successfully");
        //}
        //else {
        //    Serial.println("⚠️ Failed to decode arena settings");
        //}
    }

    void handlePositionRequest(const uint8_t* buffer, size_t len) {
        Serial.println("Position request received");

        // Send current fix data back to requester if we have valid fix
        if (fix.type >= 1) { // At least 2D fix
            sendPositionResponse();
        }
        else {
            Serial.println("⚠️ No valid fix to send");
        }
    }

    void handleSIVMessage(const uint8_t* buffer, size_t len) {
        if (len >= 2) {
            uint8_t remoteSIV = buffer[1];
            Serial.printf("Remote SIV update: %u satellites\n", remoteSIV);
            // Could store this for display or comparison
        }
    }

    void handleUnknownMessage(const uint8_t* buffer, size_t len) {
        Serial.printf("❓ Unknown packet: type=0x%02X, len=%u, from node %u\n",
            buffer[0], len, radio.getSenderId());

        // Optional: Log first few bytes for debugging
        if (len > 1) {
            Serial.print("   Data: ");
            for (size_t i = 1; i < min(len, (size_t)8); i++) {
                Serial.printf("%02X ", buffer[i]);
            }
            if (len > 8) Serial.print("...");
            Serial.println();
        }
    }

    void sendPositionResponse() {
        // Create position response packet
        struct PositionResponse {
            uint8_t msgType = MSG_REQ_POS;
            float lat;
            float lon;
            float elevation;
            uint8_t fixType;
            uint8_t siv;
            float hdop;
        } __attribute__((packed));

        PositionResponse response;
        response.lat = fix.lat;
        response.lon = fix.lon;
        response.elevation = fix.elevation;
        response.fixType = fix.type;
        response.siv = fix.SIV;
        response.hdop = fix.HDOP;

        // Send back to whoever requested (could be made more sophisticated)
        radio.send(radio.getSenderId(), &response, sizeof(response), false);
        Serial.println("✓ Position response sent");
    }

    void updateRTCMStats(uint16_t messageType, size_t length) {
        // Optional: Track RTCM message statistics
        static unsigned long totalRTCMBytes = 0;
        static unsigned int totalRTCMMessages = 0;

        totalRTCMBytes += length;
        totalRTCMMessages++;

        // Log stats periodically
        static unsigned long lastStatsMs = 0;
        if (millis() - lastStatsMs >= 30000) { // Every 30 seconds
            Serial.printf("📊 RTCM Stats: %u messages, %lu bytes total\n",
                totalRTCMMessages, totalRTCMBytes);
            lastStatsMs = millis();
        }
    }

    void loop() {
        readConsole();      // your menu/console

        // Process GNSS data and send periodic updates
        processGNSSData();

        // Handle incoming radio packets
        processRadioPackets();

        yield();
    }