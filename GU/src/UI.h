// UI.h
#pragma once

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// --- Web server configuration
WebServer webserver(80);
const char* ap_ssid = "RTK-Rover";
const char* ap_password = "11111111";  // At least 8 characters
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

int rtcmLogIndex = 0;

extern GNSSFix fix;

struct RTCMLogEntry {
    unsigned long time;
    uint16_t type;
    uint16_t bytes;
};

#define MAX_RTCM_LOG 20
RTCMLogEntry rtcmLog[MAX_RTCM_LOG] = { 0 };

// Status tracking
struct WebStatus {
    unsigned long lastUpdate = 0;
    uint32_t totalRTCMMessages = 0;
    uint32_t totalRTCMBytes = 0;
    uint8_t  lastRSSI = 0;
    bool     radioConnected = false;
    String   lastError;
} webStatus;

// ---------------- API HANDLERS ----------------

void handleAPIRTCM() {
    DynamicJsonDocument doc(2048);
    JsonArray messages = doc.createNestedArray("recentMessages");

    for (int i = 0; i < MAX_RTCM_LOG; i++) {
        int idx = (rtcmLogIndex + i) % MAX_RTCM_LOG;
        if (rtcmLog[idx].time > 0) {
            JsonObject msg = messages.createNestedObject();
            msg["time"] = String(rtcmLog[idx].time / 1000) + "s";
            msg["type"] = rtcmLog[idx].type;
            msg["bytes"] = rtcmLog[idx].bytes;
        }
    }

    String response;
    serializeJson(doc, response);
    webserver.send(200, "application/json", response);
}

void handleRoot() {
    String html;
    html.reserve(9000); // Reserve space

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
    html += ".bullwrap{position:relative;width:300px;height:300px;flex-shrink:0}";
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
    html += "<div class='bullwrap'>";
    html += "<svg width='300' height='300' id='bullseye'>";
    html += "  <g id='bullseye-rings'></g>";
    html += "  <line x1='140' y1='150' x2='160' y2='150' stroke='#333' stroke-width='2'/>";
    html += "  <line x1='150' y1='140' x2='150' y2='160' stroke='#333' stroke-width='2'/>";
    html += "  <polyline id='position-path' fill='none' stroke='#9e9e9e' stroke-width='1' stroke-linecap='round' stroke-linejoin='round'></polyline>";
    html += "  <g id='position-points'></g>";
    html += "  <circle id='current-pos' cx='150' cy='150' r='5' fill='#2196F3' stroke='#fff' stroke-width='2'/>";
    html += "</svg>";
    html += "</div>";
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
    html += "const bull={cx:150,cy:150,pxPerMeter:14};"; // 10 m -> 140 px outer ring
    html += "const ringMeters=[10,5,2,1,0.5];";

    // drawBullseyeRings
    html += "function drawBullseyeRings(){const g=document.getElementById('bullseye-rings');g.innerHTML='';";
    html += "const ringStrokes=['#ffebee','#ffcdd2','#ef9a9a','#e57373','#f44336'];";
    html += "ringMeters.forEach((m,i)=>{const r=m*bull.pxPerMeter;";
    html += "const c=document.createElementNS('http://www.w3.org/2000/svg','circle');";
    html += "c.setAttribute('cx',bull.cx);c.setAttribute('cy',bull.cy);c.setAttribute('r',r);";
    html += "c.setAttribute('fill','none');c.setAttribute('stroke',ringStrokes[i]||'#ddd');c.setAttribute('stroke-width','2');g.appendChild(c);";
    html += "const t=document.createElementNS('http://www.w3.org/2000/svg','text');";
    html += "t.setAttribute('x',bull.cx + r + 6);t.setAttribute('y',bull.cy + 4);";
    html += "t.setAttribute('font-size','10');t.setAttribute('fill','#666');t.setAttribute('text-anchor','start');";
    html += "t.textContent=m+'m';g.appendChild(t);";
    html += "const tick=document.createElementNS('http://www.w3.org/2000/svg','line');";
    html += "tick.setAttribute('x1',bull.cx + r);tick.setAttribute('y1',bull.cy - 4);";
    html += "tick.setAttribute('x2',bull.cx + r);tick.setAttribute('y2',bull.cy + 4);";
    html += "tick.setAttribute('stroke','#666');tick.setAttribute('stroke-width','1');g.appendChild(tick);});}";

    // updateStatus
    html += "function updateStatus(){fetch('/api/status').then(r=>r.json()).then(data=>{";
    html += "document.getElementById('fix-type').textContent=data.fixType;";
    html += "document.getElementById('fix-type').className='metric-value fix-'+data.fixType;";
    html += "document.getElementById('satellites').textContent=data.satellites;";
    html += "document.getElementById('hdop').textContent=(+data.hdop).toFixed(2);";
    html += "document.getElementById('rssi').textContent=data.rssi;";
    html += "document.getElementById('latitude').textContent=(+data.latitude).toFixed(6);";
    html += "document.getElementById('longitude').textContent=(+data.longitude).toFixed(6);";
    html += "document.getElementById('elevation').textContent=(+data.elevation).toFixed(2);";
    html += "document.getElementById('rtcm-messages').textContent=data.rtcmMessages;";
    html += "document.getElementById('rtcm-bytes').textContent=data.rtcmBytes;";
    html += "document.getElementById('rtcm-rate').textContent=(+data.rtcmRate).toFixed(1);";
    html += "document.getElementById('radio-status').textContent=data.radioConnected?'Connected':'Disconnected';";
    html += "updatePositionPlot(+data.latitude,+data.longitude,parseInt(data.fixType,10));";
    html += "document.getElementById('accuracy-estimate').textContent=estimateAccuracy(+data.hdop,parseInt(data.fixType,10)).toFixed(2);});}";

    // updatePositionPlot
    html += "function updatePositionPlot(lat,lon,fixType){";
    html += "if(!isFinite(lat)||!isFinite(lon)||lat===0&&lon===0)return;";
    html += "if(centerLat===null){centerLat=lat;centerLon=lon;}";
    html += "positionHistory.push({lat:lat,lon:lon,fixType:fixType,timestamp:Date.now()});";
    html += "if(positionHistory.length>maxPoints)positionHistory.shift();";
    html += "positionHistory=positionHistory.filter(p=>p.timestamp>Date.now()-300000);";
    html += "plotPositions();}";

    // plotPositions with connecting line
    html += "function plotPositions(){const ptsGroup=document.getElementById('position-points');";
    html += "const path=document.getElementById('position-path');";
    html += "ptsGroup.innerHTML='';path.setAttribute('points','');";
    html += "if(positionHistory.length===0)return;";
    html += "const pointsAttr=[];";
    html += "positionHistory.forEach((point,index)=>{";
    html += "const dLat=(point.lat-centerLat)*111320;";
    html += "const dLon=(point.lon-centerLon)*111320*Math.cos(centerLat*Math.PI/180);";
    html += "const x=bull.cx + dLon*bull.pxPerMeter;";
    html += "const y=bull.cy - dLat*bull.pxPerMeter;";
    html += "if(Math.abs(x-bull.cx) > ringMeters[0]*bull.pxPerMeter + 5 || Math.abs(y-bull.cy) > ringMeters[0]*bull.pxPerMeter + 5) return;";
    html += "pointsAttr.push(`${x},${y}`);";
    html += "const circle=document.createElementNS('http://www.w3.org/2000/svg','circle');";
    html += "circle.setAttribute('cx',x);circle.setAttribute('cy',y);";
    html += "circle.setAttribute('r',index===positionHistory.length-1?'4':'2');";
    html += "let color='#757575';";
    html += "if(point.fixType===4||point.fixType===5)color='#4CAF50';";
    html += "else if(point.fixType===2)color='#FF9800';";
    html += "else if(point.fixType===1)color='#2196F3';";
    html += "circle.setAttribute('fill',color);circle.setAttribute('stroke','#fff');circle.setAttribute('stroke-width','1');";
    html += "ptsGroup.appendChild(circle);";
    html += "});";
    html += "if(pointsAttr.length>=2){path.setAttribute('points',pointsAttr.join(' '));}";
    html += "document.getElementById('point-count').textContent=positionHistory.length;";
    html += "if(positionHistory.length>1){const timeSpan=(positionHistory[positionHistory.length-1].timestamp-positionHistory[0].timestamp)/1000;";
    html += "document.getElementById('time-span').textContent=Math.round(timeSpan);}";
    html += "}";

    // accuracy estimate
    html += "function estimateAccuracy(hdop,fixType){let baseAccuracy=5;";
    html += "if(fixType===4)baseAccuracy=0.02;else if(fixType===5)baseAccuracy=0.5;";
    html += "else if(fixType===2)baseAccuracy=1;else if(fixType===1)baseAccuracy=3;";
    html += "return baseAccuracy*(hdop>0?hdop:1);}";

    // clear history
    html += "function clearPositions(){positionHistory=[];centerLat=null;centerLon=null;";
    html += "document.getElementById('position-points').innerHTML='';";
    html += "document.getElementById('position-path').setAttribute('points','');";
    html += "document.getElementById('point-count').textContent='0';";
    html += "document.getElementById('time-span').textContent='0';}";

    // rtcm log
    html += "function updateRTCMLog(){fetch('/api/rtcm').then(r=>r.json()).then(data=>{";
    html += "document.getElementById('rtcm-log').innerHTML=data.recentMessages.map(msg=>";
    html += "`<div>${msg.time} - Type ${msg.type}: ${msg.bytes} bytes</div>`).join('');});}";

    // init
    html += "drawBullseyeRings();";
    html += "setInterval(updateStatus,2000);setInterval(updateRTCMLog,5000);";
    html += "updateStatus();updateRTCMLog();";
    html += "</script></body></html>";

    webserver.send(200, "text/html", html);
}

void handleAPIPosition() {
    DynamicJsonDocument doc(512);

    doc["latitude"] = fix.lat;
    doc["longitude"] = fix.lon;
    doc["elevation"] = fix.elevation;
    doc["fixType"] = fix.type;
    doc["timestamp"] = millis();

    String response;
    serializeJson(doc, response);
    webserver.send(200, "application/json", response);
}

void handleAPIStatus() {
    DynamicJsonDocument doc(1024);

    doc["fixType"] = fix.type;
    doc["satellites"] = fix.SIV;
    doc["hdop"] = fix.HDOP;
    doc["latitude"] = fix.lat;
    doc["longitude"] = fix.lon;
    doc["elevation"] = fix.elevation;
    doc["rssi"] = webStatus.lastRSSI;
    doc["rtcmMessages"] = webStatus.totalRTCMMessages;
    doc["rtcmBytes"] = webStatus.totalRTCMBytes;

    // Calculate data rate (bytes/sec) over the last 5s
    static unsigned long lastCalc = 0;
    static uint32_t lastBytes = 0;
    float dataRate = 0;

    unsigned long now = millis();
    if (now - lastCalc >= 5000) {
        uint32_t delta = webStatus.totalRTCMBytes - lastBytes;
        dataRate = delta / 5.0f;
        lastBytes = webStatus.totalRTCMBytes;
        lastCalc = now;
    }
    doc["rtcmRate"] = dataRate;

    doc["radioConnected"] = webStatus.radioConnected;
    doc["uptime"] = millis() / 1000;

    String response;
    serializeJson(doc, response);
    webserver.send(200, "application/json", response);
}

// ---------------- SERVER SETUP ----------------

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
    webserver.on("/", handleRoot);
    webserver.on("/api/status", handleAPIStatus);
    webserver.on("/api/position", handleAPIPosition);
    webserver.on("/api/rtcm", handleAPIRTCM);

    webserver.begin();
    Serial.println("Web server started");
}

// ---------------- HELPERS ----------------

void logRTCMMessage(uint16_t type, uint16_t bytes) {
    rtcmLog[rtcmLogIndex] = { millis(), type, bytes };
    rtcmLogIndex = (rtcmLogIndex + 1) % MAX_RTCM_LOG;

    webStatus.totalRTCMMessages++;
    webStatus.totalRTCMBytes += bytes;
}
