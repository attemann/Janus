//RTKBase.ino - Updated for Ring Buffer GNSS Module
#include <Arduino.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <RFM69.h>

#include <RTKF3F.h>
#include "RTKBase.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#define APPNAME "RTKBase 1.0"
#define THIS_NODE_ID NODEID_RTKBASE
#define COMMANDDELAY 1000  // ms to wait for GNSS command response

inline constexpr int8_t RFM69_MISO = 13;
inline constexpr int8_t RFM69_MOSI = 11;
inline constexpr int8_t RFM69_CSS = 10;
inline constexpr int8_t RFM69_SCK = 12;
inline constexpr int8_t RFM69_RST  = -1;
inline constexpr int8_t RFM69_IRQ = 8;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);

//ConfigManager config(APPNAME);

// GNSS
#define GNSS_BAUD 115200
#define UART_RX   18
#define UART_TX   17
HardwareSerial SerialGNSS(2);
GNSSModule gnss(SerialGNSS);

// RADIO
RadioModule Radio(RFM69_CSS, RFM69_IRQ, true);

unsigned int lastSpeakMs = 0;
#define SPEAK_INTERVAL_MS 10000
unsigned long msOperStart = 0;
GNSSFix fix, prevFix;  // Updated to use new GNSSFix struct
DeviceState deviceState = DeviceState::DEVICE_STARTING;


void haltUnit(const String& msg1, const String& msg2) {
    Serial.print(msg1); Serial.print(": "); Serial.println(msg2);
    sendToDisplay(msg1, msg2);
    while (true) delay(100);
}

// Print heap and stack status
void printMemoryStatus(const char* tag) {
    Serial.printf("\n--- Memory status: %s ---\n", tag);
    Serial.printf("Free heap: %ld bytes\n", esp_get_free_heap_size());
    Serial.printf("Min free heap ever: %ld bytes\n", esp_get_minimum_free_heap_size());
    Serial.printf("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("Loop task stack watermark: %u words (~%u bytes)\n",
        watermark, watermark * sizeof(StackType_t));
    Serial.println("-----------------------------");
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

void setup() {

    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 10000,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    Wire.end(); Wire.begin();
    SPI.end(); SPI.begin();
    // Bring up Serial without blocking forever
    Serial.begin(115200);
	delay(500);
    Serial.printf("%s booting\r\n", APPNAME);

    printMemoryStatus("After Serial()");


    //
    // Start display
    //
    lcdBegin();
    sendToDisplay(APPNAME, "Starting");
    printMemoryStatus("After Display()");

    //
    // Start GNSS
    //
    gnss.begin(GNSS_BAUD, UART_RX, UART_TX);
    delay(1000);
    while (SerialGNSS.available()) {
        SerialGNSS.read();
    }
    printMemoryStatus("After GNSS Start()");

    if (!gnss.sendWait("unlog", "response: OK", COMMANDDELAY)) haltUnit("GNSS", "Unlog failed");
    delay(500);

    //
    // Set up Radio
    //
    printMemoryStatus("Before Radio()");

    if (!Radio.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK, THIS_NODE_ID, NETWORK_ID, FREQUENCY_RTCM)) {
        haltUnit("Radio", "Config failed");
    }
    Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
        static_cast<uint8_t>(DEVICE_STARTING));

    printMemoryStatus("After Radio()");


    gnss.setRadio(&Radio);  

    deviceState = DeviceState::DEVICE_STARTING;
    printMemoryStatus("Before loop()");

}

void loop() {
    esp_task_wdt_reset();

    // Monitor stack usage
    //static uint32_t lastStackCheck = 0;
    //if (millis() - lastStackCheck > 5000) {
    //    UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
    //    if (watermark < 500) {  // Less than 500 words (~2KB) remaining
    //        Serial.printf("WARNING: Low stack! %u words remaining\n", watermark);
    //    }
    //    lastStackCheck = millis();
    //}

    switch (deviceState) {
    case DeviceState::DEVICE_STARTING:
        sendToDisplay("Starting", "Getting fix");
        if (!gnss.sendWait("unlog"               , "response: OK", COMMANDDELAY)) GDBG_PRINTLN("unlog failed");
        if (!gnss.sendWait("config pvtalg multi" , "response: OK", COMMANDDELAY)) GDBG_PRINTLN("config pvtalg multi");
        if (!gnss.sendWait("mode rover"          , "response: OK", COMMANDDELAY)) GDBG_PRINTLN("mode rover failed");
        if (!gnss.sendWait("gpgga com2 1"        , "response: OK", COMMANDDELAY)) GDBG_PRINTLN("gpgga com2 1 failed");
        if (!gnss.sendWait("saveconfig"          , "response: OK", COMMANDDELAY)) GDBG_PRINTLN("saveconfig failed");

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

			gnss.printFix(fix); // Debug print

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
                if (!gnss.sendWait("unlog"                          , "response: OK", COMMANDDELAY) ||
                    !gnss.sendWait("mode base time 120"             , "response: OK", COMMANDDELAY) ||
                    !gnss.sendWait("config antennadeltahen 0 0 2.0" , "response: OK", COMMANDDELAY) ||
                    !gnss.sendWait("saveconfig"                     , "response: OK", COMMANDDELAY)) haltUnit("GNSS", "Start failed");
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

            if (millis() - lastSpeakMs >= SPEAK_INTERVAL_MS) {
                //Serial.printf("BASE_SURVEYING: Remaining: %lus\n", (unsigned long)remaining);
				sendToDisplay("Surveying", String(remaining) + " sec left");
                uint8_t speakLeft = (remaining > 255) ? 255 : (uint8_t)remaining;
                Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_SURVEY, speakLeft);
                lastSpeakMs = millis();
            }
        }
        else {
            sendToDisplay("Configuring GPS", "RTK parameters");
            if (!gnss.sendWait("unlog"           , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1006 com2 10", "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1074 com2 1" , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1084 com2 1" , "response: OK", COMMANDDELAY) ||
                !gnss.sendWait("rtcm1230 com2 10", "response: OK", COMMANDDELAY)) haltUnit("GNSS", "RTCM setup failed");

            delay(500);

            Radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_DEVICESTATE,
                static_cast<uint8_t>(DeviceState::DEVICE_OPERATING));

            sendToDisplay(APPNAME, "Run 00:00:00");
            gnss.clearUARTBuffer();
            msOperStart = millis();
            deviceState = DeviceState::DEVICE_OPERATING;
        }
        break;
    }

    case DeviceState::DEVICE_OPERATING: {

        unsigned long seconds = (millis()-msOperStart) / 1000;
        unsigned long minutes = seconds / 60;
        unsigned long hours = minutes / 60;
		char timeStr[16];

		snprintf(timeStr, sizeof(timeStr), "%02lu:%02lu:%02lu", hours, minutes % 60, seconds % 60);

        sendToDisplay("Operating", timeStr);

        // Process RTCM data continuously
        gnss.pumpRTCM();

        // Print radio utilization every 30 seconds
        static unsigned long lastUtilReport = 0;
        if (millis() - lastUtilReport > 30000) {
            //Radio.printRadioUtilization();
            Radio.printRTCMTypeReport();
            lastUtilReport = millis();
        }
        break;
    }
    default:
        Serial.println("Unknown state");
        break;
    }

    //monitorSystem();

}

