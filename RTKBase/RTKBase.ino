//RTKBase.ino - Updated for Ring Buffer GNSS Module
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <RFM69.h>

#include <RTKF3F.h>
#include <RadioModule.h>
#include "RTKBase.h"

#define APPNAME "RTKBase 1.0"
#define THIS_NODE_ID NODEID_RTKBASE
#define COMMANDDELAY 1000  // ms to wait for GNSS command response

inline constexpr int8_t  RFM69_IRQ = 8;
inline constexpr int8_t  RFM69_CSS = 10;
inline constexpr int8_t  RFM69_SCK = 12;
inline constexpr int8_t  RFM69_MISO = 13;
inline constexpr int8_t  RFM69_MOSI = 11;
inline constexpr int8_t  RFM69_RST = -1;

// RADIO
RadioModule Radio(RFM69_CSS, RFM69_IRQ, true);

ConfigManager config(APPNAME);

#define GNSS_BAUD 115200
#define UART_RX   18
#define UART_TX   17

HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

GNSSFix fix, prevFix;  // Updated to use new GNSSFix struct

unsigned int lastSpeakMs = 0;
#define SPEAK_INTERVAL_MS 10000
DeviceState deviceState = DeviceState::DEVICE_STARTING;

void updateDisplayCountdown(int timeLeft) {
    static uint32_t lastUpdate = 0;
    static int lastTimeLeft = -1;

    uint32_t now = millis();

    // Only update if time changed AND at least 500ms passed
    if (timeLeft != lastTimeLeft && (now - lastUpdate >= 500)) {
        char line1[] = "Surveying";
        char line2[17];
        snprintf(line2, 17, "Time left: %ds", timeLeft);
        sendToDisplay(line1, line2);  // Use char arrays instead of String
        lastTimeLeft = timeLeft;
        lastUpdate = now;
    }
}

// Remove the old handleParsed function - not needed with new API

void haltUnit(const String& msg1, const String& msg2) {
    Serial.print(msg1); Serial.print(": "); Serial.println(msg2);
    sendToDisplay(msg1, msg2);
    while (true) delay(100);
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    delay(100);
    Serial.printf("%s booting\r\n", APPNAME);

    // Start display task
    lcdBegin();

    if (!Radio.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        sendToDisplay("Radio task", "start failed");
        while (true) delay(100);
    }

    Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
        static_cast<uint8_t>(DEVICE_STARTING));

    sendToDisplay(APPNAME, "Starting");

    gnss.begin(GNSS_BAUD, UART_RX, UART_TX, &Radio);
    if (!gnss.sendWait("unlog"        , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("versiona com2", "response: OK", COMMANDDELAY)) haltUnit("GNSS", "Config failed");

    deviceState = DeviceState::DEVICE_STARTING;
}

void loop() {

    switch (deviceState) {
    case DeviceState::DEVICE_STARTING:
        sendToDisplay("Starting", "Getting fix");
        if (!gnss.sendWait("unlog"               , "response: OK", COMMANDDELAY) ||
            !gnss.sendWait("config signalgroup 2", "response: OK", COMMANDDELAY) ||
            !gnss.sendWait("config pvtalg multi" , "response: OK", COMMANDDELAY) ||
            !gnss.sendWait("mode rover"          , "response: OK", COMMANDDELAY) ||
            !gnss.sendWait("gpgga com2 1"        , "response: OK", COMMANDDELAY) ||
            !gnss.sendWait("saveconfig"          , "response: OK", COMMANDDELAY)) haltUnit("GNSS", "Start failed");
        Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
            static_cast<uint8_t>(DeviceState::DEVICE_GETTINGFIX));
        deviceState = DeviceState::DEVICE_GETTINGFIX;
        break;

    case DeviceState::DEVICE_GETTINGFIX:
        if (gnss.pumpGGA(fix)) {
            // Create display strings
            char line1[] = "Getting fix";
            char line2[17];
            snprintf(line2, 17, "> %d satellites", fix.SIV);
            sendToDisplay(line1, line2);

            // Print fix info
            Serial.printf("GGA OK: Time=%02d:%02d:%.2f FIX=%u SIV=%d HDOP=%.2f lat=%.6f lon=%.6f elev=%.2f\n",
                fix.hour, fix.minute, fix.second, (unsigned)fix.type, fix.SIV, fix.HDOP, fix.lat, fix.lon, fix.elevation);

            // Send satellite count if changed
            if (prevFix.SIV != fix.SIV) {
                delay(200);
                Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_CD, MSG_SIV,
                    static_cast<uint8_t>(fix.SIV));
            }

			// Check if we have a fix, sufficient satellites
            if (fix.type > 0 && fix.SIV > 8) {
                Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
                    static_cast<uint8_t>(DeviceState::DEVICE_SURVEYING));
                if (!gnss.sendWait("unlog"            , "response: OK", COMMANDDELAY) ||
                    !gnss.sendWait("mode base time 15", "response: OK", COMMANDDELAY) ||
                    !gnss.sendWait("saveconfig"       , "response: OK", COMMANDDELAY)) haltUnit("GNSS", "Start failed");
                deviceState = DeviceState::DEVICE_SURVEYING;
            }
            prevFix = fix;

            // Send periodic status
            if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
                Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SIV, fix.SIV);
                lastSpeakMs = millis();
            }
        }
        break;

    case DeviceState::DEVICE_SURVEYING: {
        static uint32_t surveyStartMs = millis();
        const uint32_t elapsed = millis() - surveyStartMs;
        if (elapsed < SURVEYINTIME) {
            uint32_t remaining = (SURVEYINTIME - elapsed) / 1000;
            updateDisplayCountdown((int)remaining);

            if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
                Serial.printf("BASE_SURVEYING: Remaining: %lus\n", (unsigned long)remaining);
                uint8_t speakLeft = (remaining > 255) ? 255 : (uint8_t)remaining;
                Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SURVEY, speakLeft);
                lastSpeakMs = millis();
            }
        }
        else {
            sendToDisplay("Configuring GPS", "RTK parameters");
            if (!gnss.sendWait("unlog"           , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1006 com2 10", "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1033 com2 60", "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1074 com2 2" , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1084 com2 2" , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1094 com2 5" , "response: OK", COMMANDDELAY) ||
                //gnss.sendWait("rtcm1124 com2 5", "response: OK", COMMANDDELAY) ||
				gnss.sendWait("saveconfig"       , "response: OK", COMMANDDELAY)) haltUnit("GNSS", "RTCM setup failed");
            delay(1000);

            Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
                static_cast<uint8_t>(DeviceState::DEVICE_OPERATING));

            gnss.clearUARTBuffer();
            sendToDisplay(APPNAME, "Operating");
            deviceState = DeviceState::DEVICE_OPERATING;
        }
        break;
    }

    case DeviceState::DEVICE_OPERATING:
        // Process RTCM data continuously
        gnss.pumpRTCM();

        // Print radio utilization every 30 seconds
        static unsigned long lastUtilReport = 0;
        if (millis() - lastUtilReport > 30000) {
            Radio.printRadioUtilization();
            Radio.printRTCMTypeReport();
            lastUtilReport = millis();
        }

        // Monitor buffer usage
        //static uint32_t lastBufferCheck = 0;
        //if (millis() - lastBufferCheck > 10000) {
        //    lastBufferCheck = millis();
        //    size_t used = gnss.getBufferUsage();
        //    size_t total = used + gnss.getBufferFree();
        //    Serial.printf("GNSS Buffer: %d/%d bytes (%.1f%%)\n", used, total, (float)used * 100.0 / total);
        //    if (used > total * 0.8) {  // 80% full warning
        //        Serial.println("Warning: GNSS buffer high usage");
        //    }
        //}
        break;

    default:
        Serial.println("Unknown state");
        break;
    }
}

// Optional: Add buffer monitoring function
void monitorSystem() {
    static uint32_t lastCheck = 0;

    if (millis() - lastCheck > 5000) {
        lastCheck = millis();

        // Check GNSS buffer
        size_t bufferUsed = gnss.getBufferUsage();
        size_t bufferTotal = bufferUsed + gnss.getBufferFree();
        float bufferPercent = (float)bufferUsed * 100.0 / bufferTotal;

        Serial.printf("System Status:\n");
        Serial.printf("GNSS Buffer: %zu/%zu bytes (%.1f%%)\n",
            bufferUsed, bufferTotal, (float)bufferUsed * 100.0 / bufferTotal);
        Serial.printf("  Free Heap: %lu bytes\n", ESP.getFreeHeap());
        Serial.printf("  Min Free Heap: %lu bytes\n", ESP.getMinFreeHeap());

        // Warnings
        if (bufferPercent > 75.0) {
            Serial.println("  WARNING: GNSS buffer high usage");
        }

        if (ESP.getFreeHeap() < 20000) {
            Serial.println("  WARNING: Low memory");
        }
    }
}