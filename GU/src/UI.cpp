// UI.cpp
#include "UI.h"


WebServer webserver(80);
const char* ap_ssid = "RTK-Rover";
const char* ap_password = "11111111";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);


int rtcmLogIndex = 0;
RTCMLogEntry rtcmLog[MAX_RTCM_LOG] = { 0 };
WebStatus webStatus;


void handleRoot() {
    String html;
    html.reserve(14000);

    html += "<!DOCTYPE html><html><head>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>RTK Rover</title>";
    html += "<style>";
    html += "body{font-family:sans-serif;background:#111;color:#eee;padding:20px;}";
    html += "svg{margin:20px auto;display:block;}";
    html += "circle{stroke:#444;fill:none;}";
    html += ".decimeter{stroke:#333;stroke-width:0.5}";
    html += ".meter{stroke:#555;stroke-width:1}";
    html += "#position{stroke:white;stroke-width:1}";
    html += "#trace{fill:none;stroke:#888;stroke-width:1}";
    html += "button{margin:10px;padding:6px 12px;font-size:14px;}";
    html += "#info{text-align:center;margin-top:10px;font-size:14px;color:#ccc;}";
    html += "</style>";
    html += "</head><body>";

    html += "<h1 style='text-align:center;'>RTK Rover Bullseye</h1>";
    html += "<svg width='300' height='300' viewBox='0 0 300 300'>";
    html += "<g id='rings'></g>";
    html += "<polyline id='trace' points='' />";
    html += "<circle id='position' cx='150' cy='150' r='4' fill='red' />";
    html += "<text id='legend' x='10' y='290' fill='#aaa' font-size='12'>Fix: -, Δ=0.00m</text>";
    html += "</svg>";

    html += "<div style='text-align:center;'>";
    html += "<button onclick='zoomIn()'>Zoom In</button>";
    html += "<button onclick='zoomOut()'>Zoom Out</button>";
    html += "<button onclick='centerNow()'>Re-center</button>";
    html += "</div>";
    html += "<div id='info'>Lat: -<br>Lon: -<br>Elev: - m</div>";
    html += "<div id='rtcm' style='text-align:center;margin-top:20px;font-size:13px;color:#aaa;'>";
    html += "RTCM: - msgs | - bytes | - B/s<br>Radio: -</div>";

    html += "<script>";
    html += "let cx=150,cy=150,scale=100;";  // 1 m = 100 px initially
    html += "let centerLat=null,centerLon=null;";
    html += "let lastFixLat=null,lastFixLon=null;";
    html += "let traceMeters=[];";  // Store in meters, not pixels

    // Degree to meter conversion
    html += "function deg2m(lat1,lon1,lat2,lon2){";
    html += " const dLat=(lat2-lat1)*111320;";
    html += " const dLon=(lon2-lon1)*40075000*Math.cos((lat1+lat2)/2*Math.PI/180)/360;";
    html += " return [dLat,dLon]; }";

    // Zoom functions - redraw trace when scale changes
    html += "function zoomIn(){scale*=1.5;drawRings();redrawTrace();}";
    html += "function zoomOut(){scale/=1.5;drawRings();redrawTrace();}";

    // Re-center to last valid fix
    html += "function centerNow(){";
    html += " if(lastFixLat&&lastFixLon){";
    html += "  centerLat=lastFixLat;centerLon=lastFixLon;";
    html += "  traceMeters=[];";
    html += "  document.getElementById('trace').setAttribute('points','');";
    html += "  console.log('Re-centered to last fix');";
    html += " }}";

    // Draw concentric circles
    html += "function drawRings(){";
    html += " const rings=document.getElementById('rings');";
    html += " rings.innerHTML='';";
    // Decimeter circles: 0.1m intervals
    html += " for(let r=0.1;r<=0.9;r+=0.1){";
    html += "  const px=r*scale;";
    html += "  if(px<140){";
    html += "   const c=document.createElementNS('http://www.w3.org/2000/svg','circle');";
    html += "   c.setAttribute('cx','150');c.setAttribute('cy','150');";
    html += "   c.setAttribute('r',px);c.setAttribute('class','decimeter');";
    html += "   rings.appendChild(c);";
    html += "  }}";
    // Meter circles: 1-5m
    html += " for(let r=1;r<=5;r++){";
    html += "  const px=r*scale;";
    html += "  if(px<140){";
    html += "   const c=document.createElementNS('http://www.w3.org/2000/svg','circle');";
    html += "   c.setAttribute('cx','150');c.setAttribute('cy','150');";
    html += "   c.setAttribute('r',px);c.setAttribute('class','meter');";
    html += "   rings.appendChild(c);";
    html += "  }}";
    html += "}";

    // Redraw trace with current scale
    html += "function redrawTrace(){";
    html += " const points=traceMeters.map(([dLat,dLon])=>{";
    html += "  const x=cx+dLon*scale;";
    html += "  const y=cy-dLat*scale;";
    html += "  return `${x},${y}`;";
    html += " });";
    html += " document.getElementById('trace').setAttribute('points',points.join(' '));";
    html += "}";

    // Position update with data validation
    html += "async function updatePosition(){";
    html += " try{";
    html += "  const r=await fetch('/api/position');";
    html += "  if(!r.ok)return;";
    html += "  const j=await r.json();";
    html += "  const lat=j.latitude, lon=j.longitude, elev=j.elevation, fix=j.fixType;";

    // Data validation
    html += "  if(typeof lat!=='number'||typeof lon!=='number')return;";
    html += "  if(!isFinite(lat)||!isFinite(lon))return;";
    html += "  if(lat===0&&lon===0)return;";
    html += "  if(Math.abs(lat)>90||Math.abs(lon)>180)return;";

    html += "  if(centerLat===null){";
    html += "   centerLat=lat;centerLon=lon;";
    html += "   console.log('Center initialized');";
    html += "  }";

    html += "  const [dLat,dLon]=deg2m(centerLat,centerLon,lat,lon);";
    html += "  const dist=Math.sqrt(dLat*dLat+dLon*dLon);";
    html += "  const x=cx+dLon*scale;";
    html += "  const y=cy-dLat*scale;";

    html += "  lastFixLat=lat; lastFixLon=lon;";

    // Store in meters, not pixels
    html += "  traceMeters.push([dLat,dLon]);";
    html += "  if(traceMeters.length>300)traceMeters.shift();";
    html += "  redrawTrace();";

    html += "  const pos=document.getElementById('position');";
    html += "  pos.setAttribute('cx',x);pos.setAttribute('cy',y);";

    // Update legend with distance from center
    html += "  document.getElementById('legend').textContent=`Fix: ${fix}, Δ=${dist.toFixed(2)}m`;";

    html += "  document.getElementById('info').innerHTML=";
    html += "   `Lat: ${lat.toFixed(7)}<br>Lon: ${lon.toFixed(7)}<br>Elev: ${elev.toFixed(2)} m`;";
    html += " }catch(e){console.error('update fail',e);}";
    html += "}";

    // Fetch RTCM statistics
    html += "async function fetchRTCMStats(){";
    html += " try{";
    html += "  const r=await fetch('/api/status');";
    html += "  if(!r.ok)return;";
    html += "  const j=await r.json();";
    html += "  const msgs=j.rtcmMessages||0;";
    html += "  const bytes=j.rtcmBytes||0;";
    html += "  const rate=j.rtcmRate||0;";
    html += "  const radio=j.radioConnected?'Connected':'Disconnected';";
    html += "  const radioColor=j.radioConnected?'#0f0':'#f00';";
    html += "  document.getElementById('rtcm').innerHTML=";
    html += "   `RTCM: ${msgs} msgs | ${bytes} bytes | ${rate.toFixed(1)} B/s<br>Radio: <span style='color:${radioColor}'>${radio}</span>`;";
    html += " }catch(e){console.error('rtcm stats fail',e);}";
    html += "}";

    html += "drawRings();";  // Draw rings immediately on page load
    html += "setInterval(updatePosition,500);";
    html += "setInterval(fetchRTCMStats,1000);";
    html += "</script>";

    html += "</body></html>";

    webserver.send(200, "text/html", html);
}


void handleAPIStatus() {
    JsonDocument doc;
    doc["fixType"] = fix.type;
    doc["satellites"] = fix.SIV;
    doc["hdop"] = fix.HDOP;
    doc["latitude"] = fix.lat;
    doc["longitude"] = fix.lon;
    doc["elevation"] = fix.elevation;
    doc["rssi"] = webStatus.lastRSSI;
    doc["rtcmMessages"] = webStatus.totalRTCMMessages;
    doc["rtcmBytes"] = webStatus.totalRTCMBytes;
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

void handleAPIPosition() {
    JsonDocument doc;
    doc["latitude"] = fix.lat;
    doc["longitude"] = fix.lon;
    doc["elevation"] = fix.elevation;
    doc["fixType"] = fix.type;
    doc["timestamp"] = millis();
    String response;
    serializeJson(doc, response);
    webserver.send(200, "application/json", response);
}

void handleAPIRTCM() {
    JsonDocument doc;
    JsonArray messages = doc["recentMessages"].to<JsonArray>();
    for (int i = 0; i < MAX_RTCM_LOG; i++) {
        int idx = (rtcmLogIndex + i) % MAX_RTCM_LOG;
        if (rtcmLog[idx].time > 0) {
            JsonObject msg = messages.add<JsonObject>();
            msg["time"] = String(rtcmLog[idx].time / 1000) + "s";
            msg["type"] = rtcmLog[idx].type;
            msg["bytes"] = rtcmLog[idx].bytes;
        }
    }
    String response;
    serializeJson(doc, response);
    webserver.send(200, "application/json", response);
}

void setupWebServer() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ap_ssid, ap_password);
    webserver.on("/", handleRoot);
    webserver.on("/api/status", handleAPIStatus);
    webserver.on("/api/position", handleAPIPosition);
    webserver.on("/api/rtcm", handleAPIRTCM);
    webserver.begin();
}

void logRTCMMessage(uint16_t type, uint16_t bytes) {
    rtcmLog[rtcmLogIndex] = { millis(), type, bytes };
    rtcmLogIndex = (rtcmLogIndex + 1) % MAX_RTCM_LOG;
    webStatus.totalRTCMMessages++;
    webStatus.totalRTCMBytes += bytes;
}