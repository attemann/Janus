// GU.ino
#include <Arduino.h>
#include <HardwareSerial.h>
#include "_macros.h"
#include <RFM69.h>
#include <RTKF3F.h>
#include "GU.h"

#include "Glider.h"
#include "Arena.h"

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "src/UI.h"

#define APPNAME "GU 1.0"
#define THIS_NODE_ID NODEID_GU
#define COMMANDDELAY 1000  // ms to wait for GNSS command response

// Define your ESP model (uncomment the one you're using)
#define XIAO_ESP32C6
// #define ESP32S3

// Radio pins
#if defined(XIAO_ESP32C6)
inline constexpr int8_t RFM69_CSS  = 16; // Hvit
inline constexpr int8_t RFM69_SCK  = 19; // Grå
inline constexpr int8_t RFM69_MOSI = 18; // Magenta
inline constexpr int8_t RFM69_MISO = 20; // Blå
inline constexpr int8_t RFM69_IRQ  = 17; // Gul

// UART (GNSS)
#define UART_RX   4
#define UART_TX   5

#elif defined(ESP32S3)
inline constexpr int8_t RFM69_CSS  = 10;
inline constexpr int8_t RFM69_SCK  = 12;
inline constexpr int8_t RFM69_MOSI = 11;
inline constexpr int8_t RFM69_MISO = 13;
inline constexpr int8_t RFM69_IRQ  = 14;

// UART (GNSS)
#define UART_RX   17
#define UART_TX   18

#else
#error "No ESP model defined! Please define XIAO_ESP32C6, ESP32S3, or ESP32C3"
#endif

// Common definitions (apply to all models)
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;

#define GNSS_BAUD 115200

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

RadioModule radio(RFM69_CSS, RFM69_IRQ, true);
RadioModule::RTCM_Reassembler reassembler;

Arena  arena;
Glider glider;
GNSSFix fix;
bool showFix = true;

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
    
    delay(200);
    if (!gnss.sendWait("unlog", "response: OK", COMMANDDELAY)) {
        //haltUnit("GNSS", "- No [unlog] response");
    }

    // Configure rover
    if (!gnss.sendWait("config signalgroup 2"     ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config pvtalg multi"      ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config rtk timeout 30"    ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config rtk reliability 1" ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("mode rover"               ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("gpgga com2 1"             ,"response: OK", COMMANDDELAY)) {
            Serial.println("- GPS config fail");
    } else {
        radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE, DEVICE_STARTING);
        Serial.println(APPNAME " ready!");
        showMenu();
	} 
    setupWebServer();  // Add this line
}

void processGNSSData() {
    bool res = false;
    res = gnss.pumpGGA(fix);
    if (res && showFix) {
        Serial.printf("GGA OK: Time=%02d:%02d:%.2f" ANSI_GREEN " FIX = %u " ANSI_RESET "SIV = %d HDOP = %.2f lat = %.6f lon = %.6f elev = %.2f\n",
            fix.hour, fix.minute, fix.second, (unsigned)fix.type, fix.SIV, fix.HDOP, fix.lat, fix.lon, fix.elevation);
    }
    sendPeriodicUpdate();
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
        webStatus.lastRSSI = radio.getLastRSSI();
        webStatus.radioConnected = true;
        webStatus.lastUpdate = millis();
        handleRadioMessage(buffer, len);
    }
    else {
        // Check for radio timeout
        if (millis() - webStatus.lastUpdate > 10000) {
            webStatus.radioConnected = false;
        }
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
    //Serial.printf("🔍 Full RTCM Debug: received %u bytes\n", len);

    if (len < 6) {  // Minimum RTCM frame size
        Serial.printf("⚠️ Invalid RTCM packet: too short (%u bytes, minimum 6)\n", len);
        return;
    }

    // Print raw packet for debugging
    //Serial.print("Raw packet: ");
    //for (size_t i = 0; i < min(len, (size_t)16); i++) {
    //    Serial.printf("%02X ", buffer[i]);
    //}
    //if (len > 16) Serial.print("...");
    //Serial.println();

    // Since MSG_RTCM = 0xD3, we expect raw RTCM data (no wrapper)
    if (buffer[0] != 0xD3) {
        Serial.printf("⚠️ Expected RTCM start (0xD3), got 0x%02X\n", buffer[0]);
        return;
    }

    // This IS the RTCM data (no header to skip)
    const uint8_t* rtcm = buffer;     // No skipping needed
    size_t rlen = len;                // Full length

    Serial.printf("✓ RTCM packet: %u bytes starting with 0xD3\n", rlen);

    // Extract and verify length
    uint16_t payloadLen = ((rtcm[1] & 0x03) << 8) | rtcm[2];
    size_t expectedLen = 3 + payloadLen + 3; // header + payload + CRC

    //Serial.printf("RTCM length check: payload=%u, expected total=%u, actual=%u\n",
    //    payloadLen, expectedLen, rlen);

    if (expectedLen != rlen) {
        Serial.printf("⚠️ RTCM length mismatch: expected %u, got %u\n", expectedLen, rlen);
        // Don't return here - try to process anyway in case it's just a length issue
    }

    if (!validateAndInjectRTCM(rtcm, rlen, "full")) {
        Serial.println("⚠️ Failed to process full RTCM packet");

        // Additional debugging - show what validateAndInjectRTCM saw
        Serial.println("🔍 Detailed validation check:");

        // Manual RTCM validation steps
        Serial.printf("  - Start byte: 0x%02X %s\n", rtcm[0],
            (rtcm[0] == 0xD3) ? "✓" : "✗");

        Serial.printf("  - Reserved bits: 0x%02X %s\n", rtcm[1] & 0xFC,
            ((rtcm[1] & 0xFC) == 0) ? "✓" : "✗");

        if (rlen >= 6) {
            uint16_t msgType = (rtcm[3] << 4) | (rtcm[4] >> 4);
            Serial.printf("  - Message type: %u\n", msgType);
        }

        // Try CRC check manually
        if (gnss.isValidRTCM(rtcm, rlen)) {
            Serial.println("  - GNSS validation: ✓");
        }
        else {
            Serial.println("  - GNSS validation: ✗");
        }
    }
}

void handleFragmentedRTCM(const uint8_t* buffer, size_t len) {
    if (len <= 4) {
        Serial.println("⚠️ Invalid RTCM fragment: too short");
        return;
    }

    // Extract fragment info for logging
    //uint8_t fragIndex = buffer[1];
    //uint8_t totalFrags = buffer[2];
    //uint8_t msgId = buffer[3];

    // Serial.printf("RTCM fragment %u/%u (msgId=%u)\n", fragIndex + 1, totalFrags, msgId);

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
    Serial.printf("✓ RTCM %s: type " ANSI_YELLOW "[%4u]" ANSI_RESET " %u bytes\n", source, type, (unsigned)rlen);

    logRTCMMessage(type, rlen);  // Add this line

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
    readConsole();
    processGNSSData();
    processRadioPackets();

    webserver.handleClient();

    yield();
}