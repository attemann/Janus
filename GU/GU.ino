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

#define APPNAME "GU 1.0"
#define THIS_NODE_ID NODEID_GU
#define COMMANDDELAY 1000  // ms to wait for GNSS command response

// Radio
inline constexpr int8_t RFM69_CSS  = 21; // Hvit
inline constexpr int8_t RFM69_SCK  = 19; // Grå
inline constexpr int8_t RFM69_MOSI = 18; // Magenta
inline constexpr int8_t RFM69_MISO = 20; // Blå

inline constexpr int8_t RFM69_IRQ  = 16; // Gul
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST  = -1;

bool showFix = true;

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

// Web server configuration
WebServer server(80);
const char* ap_ssid = "RTK-Rover";
const char* ap_password = "11111111";  // At least 8 characters
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Status tracking
struct WebStatus {
    unsigned long lastUpdate;
    uint32_t totalRTCMMessages;
    uint32_t totalRTCMBytes;
    uint8_t lastRSSI;
    bool radioConnected;
    String lastError;
} webStatus;

void setupWebServer() {
    // Configure Access Point
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ap_ssid, ap_password);

    Serial.println("Access Point started");
    Serial.printf("SSID: %s\n", ap_ssid);
    Serial.printf("Password: %s\n", ap_password);
    Serial.printf("IP address: %s\n", WiFi.softAPIP().toString().c_str());

    // Setup web routes
    server.on("/", handleRoot);
    server.on("/api/status", handleAPIStatus);
    server.on("/api/position", handleAPIPosition);
    server.on("/api/rtcm", handleAPIRTCM);

    server.begin();
    Serial.println("Web server started");
}

void handleRoot() {
    String html;
    html.reserve(6000); // Reserve space

    // Build HTML in chunks to avoid compiler issues
    html += "<!DOCTYPE html><html><head>";
    html += "<title>RTK Rover Status</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0}";
    html += ".container{max-width:1000px;margin:0 auto}";
    html += ".card{background:white;padding:20px;margin:10px 0;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}";
    html += ".status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px}";
    html += ".metric{text-align:center}";
    html += ".metric-value{font-size:2em;font-weight:bold;margin:10px 0}";
    html += ".metric-label{color:#666;font-size:0.9em}";
    html += ".fix-0{color:#dc3545}.fix-1{color:#fd7e14}.fix-2{color:#ffc107}.fix-4,.fix-5{color:#28a745}";
    html += ".coordinates{font-family:monospace;font-size:1.1em}";
    html += "h1{text-align:center;color:#333}h2{color:#555;margin-top:0}";
    html += "button{background:#007bff;color:white;border:none;border-radius:4px;cursor:pointer;padding:5px 10px}";
    html += "</style></head><body>";

    html += "<div class='container'><h1>RTK Rover Status</h1>";

    // GNSS Position Card
    html += "<div class='card'><h2>GNSS Position</h2>";
    html += "<div class='status-grid'>";
    html += "<div class='metric'><div class='metric-value fix-0' id='fix-type'>-</div><div class='metric-label'>Fix Type</div></div>";
    html += "<div class='metric'><div class='metric-value' id='satellites'>-</div><div class='metric-label'>Satellites</div></div>";
    html += "<div class='metric'><div class='metric-value' id='hdop'>-</div><div class='metric-label'>HDOP</div></div>";
    html += "<div class='metric'><div class='metric-value' id='rssi'>-</div><div class='metric-label'>Radio RSSI</div></div>";
    html += "</div>";
    html += "<div style='margin-top:20px'><div class='coordinates'>";
    html += "<div>Latitude: <span id='latitude'>-</span></div>";
    html += "<div>Longitude: <span id='longitude'>-</span></div>";
    html += "<div>Elevation: <span id='elevation'>-</span> m</div>";
    html += "</div></div></div>";

    // RTCM Data Card
    html += "<div class='card'><h2>RTCM Data Stream</h2>";
    html += "<div style='display:flex;justify-content:space-between;flex-wrap:wrap'>";
    html += "<div style='text-align:center;flex:1;min-width:120px'>";
    html += "<div class='metric-value' id='rtcm-messages'>-</div><div class='metric-label'>Messages</div></div>";
    html += "<div style='text-align:center;flex:1;min-width:120px'>";
    html += "<div class='metric-value' id='rtcm-bytes'>-</div><div class='metric-label'>Total Bytes</div></div>";
    html += "<div style='text-align:center;flex:1;min-width:120px'>";
    html += "<div class='metric-value' id='rtcm-rate'>-</div><div class='metric-label'>Bytes/sec</div></div>";
    html += "</div>";
    html += "<div style='margin-top:15px'>";
    html += "<div>Radio Status: <span id='radio-status'>-</span></div>";
    html += "<div>Base Station: Node 1</div></div></div>";

    // Position Accuracy Card (Bullseye)
    html += "<div class='card'><h2>Position Accuracy</h2>";
    html += "<div style='display:flex;align-items:center;gap:20px;flex-wrap:wrap'>";
    html += "<div style='position:relative;width:300px;height:300px;flex-shrink:0'>";
    html += "<svg width='300' height='300' id='bullseye'>";
    html += "<circle cx='150' cy='150' r='140' fill='none' stroke='#ffebee' stroke-width='2'/>";
    html += "<circle cx='150' cy='150' r='105' fill='none' stroke='#ffcdd2' stroke-width='2'/>";
    html += "<circle cx='150' cy='150' r='70' fill='none' stroke='#ef9a9a' stroke-width='2'/>";
    html += "<circle cx='150' cy='150' r='35' fill='none' stroke='#e57373' stroke-width='2'/>";
    html += "<circle cx='150' cy='150' r='10' fill='none' stroke='#f44336' stroke-width='2'/>";
    html += "<line x1='140' y1='150' x2='160' y2='150' stroke='#333' stroke-width='2'/>";
    html += "<line x1='150' y1='140' x2='150' y2='160' stroke='#333' stroke-width='2'/>";
    html += "<g id='position-points'></g>";
    html += "<circle id='current-pos' cx='150' cy='150' r='5' fill='#2196F3' stroke='#fff' stroke-width='2'/>";
    html += "</svg>";
    html += "<div style='position:absolute;top:5px;right:5px;font-size:0.8em;color:#666'>";
    html += "<div>10m</div><div>5m</div><div>2m</div><div>1m</div><div>0.5m</div></div></div>";
    html += "<div style='flex:1;min-width:200px'>";
    html += "<div class='metric'><div class='metric-value' id='accuracy-estimate'>-</div>";
    html += "<div class='metric-label'>Est. Accuracy (m)</div></div>";
    html += "<div style='margin-top:20px'>";
    html += "<div>Points: <span id='point-count'>0</span></div>";
    html += "<div>Time: <span id='time-span'>0</span>s</div>";
    html += "<button onclick='clearPositions()'>Clear</button></div></div></div></div>";

    // RTCM Log Card
    html += "<div class='card'><h2>Recent RTCM Messages</h2>";
    html += "<div id='rtcm-log' style='font-family:monospace;font-size:0.9em;max-height:200px;overflow-y:auto'>";
    html += "<div>Waiting for data...</div></div></div></div>";

    // JavaScript
    html += "<script>";
    html += "let positionHistory=[],centerLat=null,centerLon=null;const maxPoints=100;";
    html += "function updateStatus(){fetch('/api/status').then(r=>r.json()).then(data=>{";
    html += "document.getElementById('fix-type').textContent=data.fixType;";
    html += "document.getElementById('fix-type').className='metric-value fix-'+data.fixType;";
    html += "document.getElementById('satellites').textContent=data.satellites;";
    html += "document.getElementById('hdop').textContent=data.hdop.toFixed(2);";
    html += "document.getElementById('rssi').textContent=data.rssi;";
    html += "document.getElementById('latitude').textContent=data.latitude.toFixed(6);";
    html += "document.getElementById('longitude').textContent=data.longitude.toFixed(6);";
    html += "document.getElementById('elevation').textContent=data.elevation.toFixed(2);";
    html += "document.getElementById('rtcm-messages').textContent=data.rtcmMessages;";
    html += "document.getElementById('rtcm-bytes').textContent=data.rtcmBytes;";
    html += "document.getElementById('rtcm-rate').textContent=data.rtcmRate.toFixed(1);";
    html += "document.getElementById('radio-status').textContent=data.radioConnected?'Connected':'Disconnected';";
    html += "updatePositionPlot(data.latitude,data.longitude,data.fixType);";
    html += "document.getElementById('accuracy-estimate').textContent=estimateAccuracy(data.hdop,data.fixType).toFixed(2);});}";

    html += "function updatePositionPlot(lat,lon,fixType){";
    html += "if(lat===0&&lon===0)return;";
    html += "if(centerLat===null){centerLat=lat;centerLon=lon;}";
    html += "positionHistory.push({lat:lat,lon:lon,fixType:fixType,timestamp:Date.now()});";
    html += "if(positionHistory.length>maxPoints)positionHistory.shift();";
    html += "positionHistory=positionHistory.filter(p=>p.timestamp>Date.now()-300000);plotPositions();}";

    html += "function plotPositions(){const svg=document.getElementById('position-points');svg.innerHTML='';";
    html += "if(positionHistory.length===0)return;";
    html += "positionHistory.forEach((point,index)=>{";
    html += "const deltaLat=(point.lat-centerLat)*111320;";
    html += "const deltaLon=(point.lon-centerLon)*111320*Math.cos(centerLat*Math.PI/180);";
    html += "const x=150+deltaLon*14;const y=150-deltaLat*14;";
    html += "if(Math.abs(x-150)>145||Math.abs(y-150)>145)return;";
    html += "const circle=document.createElementNS('http://www.w3.org/2000/svg','circle');";
    html += "circle.setAttribute('cx',x);circle.setAttribute('cy',y);";
    html += "circle.setAttribute('r',index===positionHistory.length-1?'4':'2');";
    html += "let color='#757575';";
    html += "if(point.fixType===4||point.fixType===5)color='#4CAF50';";
    html += "else if(point.fixType===2)color='#FF9800';";
    html += "else if(point.fixType===1)color='#2196F3';";
    html += "circle.setAttribute('fill',color);circle.setAttribute('stroke','#fff');";
    html += "circle.setAttribute('stroke-width','1');svg.appendChild(circle);});";
    html += "document.getElementById('point-count').textContent=positionHistory.length;";
    html += "if(positionHistory.length>1){";
    html += "const timeSpan=(positionHistory[positionHistory.length-1].timestamp-positionHistory[0].timestamp)/1000;";
    html += "document.getElementById('time-span').textContent=Math.round(timeSpan);}}";

    html += "function estimateAccuracy(hdop,fixType){let baseAccuracy=5;";
    html += "if(fixType===4)baseAccuracy=0.02;else if(fixType===5)baseAccuracy=0.5;";
    html += "else if(fixType===2)baseAccuracy=1;else if(fixType===1)baseAccuracy=3;";
    html += "return baseAccuracy*(hdop>0?hdop:1);}";

    html += "function clearPositions(){positionHistory=[];centerLat=null;centerLon=null;";
    html += "document.getElementById('position-points').innerHTML='';";
    html += "document.getElementById('point-count').textContent='0';";
    html += "document.getElementById('time-span').textContent='0';}";

    html += "function updateRTCMLog(){fetch('/api/rtcm').then(r=>r.json()).then(data=>{";
    html += "document.getElementById('rtcm-log').innerHTML=data.recentMessages.map(msg=>";
    html += "`<div>${msg.time} - Type ${msg.type}: ${msg.bytes} bytes</div>`).join('');});}";

    html += "setInterval(updateStatus,2000);setInterval(updateRTCMLog,5000);";
    html += "updateStatus();updateRTCMLog();";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
}
void handleAPIPosition() {
    JsonDocument doc;

    doc["latitude"]  = fix.lat;
    doc["longitude"] = fix.lon;
    doc["elevation"] = fix.elevation;
    doc["fixType"]   = fix.type;
    doc["timestamp"] = millis();

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}


void handleAPIStatus() {
    JsonDocument doc;

    doc["fixType"]      = fix.type;
    doc["satellites"]   = fix.SIV;
    doc["hdop"]         = fix.HDOP;
    doc["latitude"]     = fix.lat;
    doc["longitude"]    = fix.lon;
    doc["elevation"]    = fix.elevation;
    doc["rssi"]         = webStatus.lastRSSI;
    doc["rtcmMessages"] = webStatus.totalRTCMMessages;
    doc["rtcmBytes"]    = webStatus.totalRTCMBytes;

    // Calculate data rate
    static unsigned long lastCalc = 0;
    static uint32_t lastBytes = 0;
    float dataRate = 0;

    if (millis() - lastCalc >= 5000) {
        dataRate = (webStatus.totalRTCMBytes - lastBytes) / 5.0;
        lastBytes = webStatus.totalRTCMBytes;
        lastCalc = millis();
    }

    doc["rtcmRate"] = dataRate;
    doc["radioConnected"] = webStatus.radioConnected;
    doc["uptime"] = millis() / 1000;

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

struct RTCMLogEntry {
    unsigned long time;
    uint16_t type;
    uint16_t bytes;
};

#define MAX_RTCM_LOG 20
RTCMLogEntry rtcmLog[MAX_RTCM_LOG];
int rtcmLogIndex = 0;

void logRTCMMessage(uint16_t type, uint16_t bytes) {
    rtcmLog[rtcmLogIndex] = { millis(), type, bytes };
    rtcmLogIndex = (rtcmLogIndex + 1) % MAX_RTCM_LOG;

    webStatus.totalRTCMMessages++;
    webStatus.totalRTCMBytes += bytes;
}

void handleAPIRTCM() {
    JsonDocument doc;
    JsonArray messages = doc["recentMessages"].to<JsonArray>();

    for (int i = 0; i < MAX_RTCM_LOG; i++) {
        int idx = (rtcmLogIndex + i) % MAX_RTCM_LOG;
        if (rtcmLog[idx].time > 0) {
            JsonObject msg = messages.add<JsonObject>();
            msg["time"]  = String(rtcmLog[idx].time / 1000) + "s";
            msg["type"]  = rtcmLog[idx].type;
            msg["bytes"] = rtcmLog[idx].bytes;
        }
    }

    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}


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
    

	bool gnssConnected = false;
    const char* ports[] = { "com1", "com2", "com3" };
    char cmd[32];

    for (int i = 0; i < 3; ++i) {
        snprintf(cmd, sizeof(cmd), "versiona %s", ports[i]);
        Serial.printf("Trying [%s]\n", cmd);

        if (gnss.sendWait(cmd, "#VERSIONA", COMMANDDELAY)) {
            Serial.printf("✔️  GNSS replied on %s\n", ports[i]);
            radio.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_ERROR, (uint8_t)ERR_GPS);
            gnssConnected = true;
            break;
        }
        delay(200);
    }

    delay(200);
    if (!gnssConnected) {
        Serial.println("- No GPS COM1/2/3");
        haltUnit("GNSS", "No GPS COM1/2/3");
    }

    delay(200);
    if (!gnss.sendWait("unlog", "response: OK", COMMANDDELAY)) {
        haltUnit("GNSS", "- No [unlog] response");
    }

    // Configure rover
    if (!gnss.sendWait("config signalgroup 2"     ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config pvtalg multi"      ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config rtk timeout 30"    ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("config rtk reliability 1" ,"response: OK", COMMANDDELAY) ||
        !gnss.sendWait("mode rover"               , "response: OK", COMMANDDELAY) ||
        !gnss.sendWait("gpgga com2 1"             , "response: OK" ,COMMANDDELAY)) {
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

    server.handleClient();  // Add this line

    yield();
}