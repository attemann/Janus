#include "ConfigMgr.h"
#include <WiFi.h>
#include <WebServer.h>
#include <vector>

using std::vector;

static const char* KEYLIST_KEY = "__keys";

ConfigManager::ConfigManager(const char* nvsNamespace) : _ns(nvsNamespace) {}

bool ConfigManager::begin() {
    return _pref.begin(_ns.c_str(), false);
}

bool ConfigManager::clearAll() {
    return _pref.clear();
}

bool ConfigManager::exists(const String& key) const {
    String v = const_cast<Preferences&>(_pref).getString(key.c_str(), "\x01");
    return v != "\x01";
}

bool ConfigManager::setString(const String& key, const String& value) {
    if (!_pref.putString(key.c_str(), value)) return false;
    return ensureKeyInList(key);
}

String ConfigManager::getString(const String& key, const String& def) const {
    return const_cast<Preferences&>(_pref).getString(key.c_str(), def);
}

bool ConfigManager::setInt(const String& key, int32_t value) {
    if (_pref.putInt(key.c_str(), value) == 0) return false;
    return ensureKeyInList(key);
}

int32_t ConfigManager::getInt(const String& key, int32_t def) const {
    return const_cast<Preferences&>(_pref).getInt(key.c_str(), def);
}

bool ConfigManager::setBool(const String& key, bool value) {
    if (_pref.putBool(key.c_str(), value) == 0) return false;
    return ensureKeyInList(key);
}

bool ConfigManager::getBool(const String& key, bool def) const {
    return const_cast<Preferences&>(_pref).getBool(key.c_str(), def);
}

bool ConfigManager::setFloat(const String& key, float value) {
    if (_pref.putFloat(key.c_str(), value) == 0) return false;
    return ensureKeyInList(key);
}

float ConfigManager::getFloat(const String& key, float def) const {
    return const_cast<Preferences&>(_pref).getFloat(key.c_str(), def);
}

bool ConfigManager::remove(const String& key) {
    if (!_pref.remove(key.c_str())) return false;
    return removeKeyFromList(key);
}

void ConfigManager::getKeys(vector<String>& out) const {
    loadKeyList(out);
}

void ConfigManager::loadKeyList(vector<String>& keys) const {
    keys.clear();
    String blob = const_cast<Preferences&>(_pref).getString(KEYLIST_KEY, "");
    if (blob.length() == 0) return;
    int start = 0;
    while (true) {
        int nl = blob.indexOf('\n', start);
        if (nl < 0) {
            String k = blob.substring(start);
            if (k.length()) keys.push_back(k);
            break;
        }
        String k = blob.substring(start, nl);
        if (k.length()) keys.push_back(k);
        start = nl + 1;
    }
}

bool ConfigManager::saveKeyList(const vector<String>& keys) const {
    String blob;
    for (size_t i = 0; i < keys.size(); ++i) {
        if (keys[i] == KEYLIST_KEY) continue;
        blob += keys[i];
        blob += '\n';
    }
    return const_cast<Preferences&>(_pref).putString(KEYLIST_KEY, blob) > 0;
}

bool ConfigManager::ensureKeyInList(const String& key) {
    if (key == KEYLIST_KEY) return true;
    vector<String> keys;
    loadKeyList(keys);
    for (const auto& k : keys)
        if (k == key) return true;
    keys.push_back(key);
    return saveKeyList(keys);
}

bool ConfigManager::removeKeyFromList(const String& key) {
    vector<String> keys;
    loadKeyList(keys);
    vector<String> out;
    for (const auto& k : keys)
        if (k != key) out.push_back(k);
    return saveKeyList(out);
}

// ---------- Admin WiFi ----------
void ConfigManager::startAdminWindow(const AdminOpts& opts) {
    if (_adminActive) stopAdmin();

    _ssid = opts.ssid;
    _pwd = opts.password;
    _apIP = opts.apIP ? opts.apIP : IPAddress(192, 168, 4, 1);
    _apChannel = opts.channel > 0 ? opts.channel : 1;

    if (_ssid.length() == 0) {
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char buf[24];
        snprintf(buf, sizeof(buf), "Janus%02X%02X", mac[4], mac[5]);
        _ssid = buf;
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(_apIP, _apIP, IPAddress(255, 255, 255, 0));
    bool ok = false;
    if (_pwd.length() >= 8)
        ok = WiFi.softAP(_ssid.c_str(), _pwd.c_str(), _apChannel);
    else
        ok = WiFi.softAP(_ssid.c_str(), nullptr, _apChannel);
    if (!ok) return;

    _server = new WebServer(80);
    setupHttp();

    uint32_t dur = opts.windowMs ? opts.windowMs : 120000;
    _adminEndMs = millis() + dur;
    _adminActive = true;

    Serial.printf("🔧 Admin-AP '%s' %s. IP: %s  (slår seg av om %u s)\n",
        _ssid.c_str(),
        _pwd.length() >= 8 ? "(med passord)" : "(åpen)",
        _apIP.toString().c_str(), dur / 1000);
}

void ConfigManager::stopAdmin() {
    if (_server) {
        _server->stop();
        delete _server;
        _server = nullptr;
    }
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _adminActive = false;
    Serial.println("🔒 Admin-AP stoppet og WiFi slått av.");
}

void ConfigManager::loop() {
    if (_adminActive && _server) {
        _server->handleClient();
        if ((int32_t)(millis() - _adminEndMs) >= 0) {
            stopAdmin();
        }
    }
}

void ConfigManager::setupHttp() {
    _server->on("/", HTTP_GET, [this]() { handleIndex(); });
    _server->on("/save", HTTP_POST, [this]() { handleSave(); });
    _server->on("/delete", HTTP_POST, [this]() { handleDelete(); });
    _server->on("/export", HTTP_GET, [this]() { handleExport(); });
    _server->on("/import", HTTP_POST, [this]() { handleImport(); });

    // Captive portal-detektering
    _server->on("/generate_204", HTTP_GET, [this]() { handleCaptive(); });
    _server->on("/fwlink", HTTP_GET, [this]() { handleCaptive(); });
    _server->onNotFound([this]() { handleCaptive(); });

    _server->begin();
}

void ConfigManager::handleCaptive() {
    _server->sendHeader("Location", "http://192.168.4.1/", true);  // fast IP
    _server->send(302, "text/plain", "");
}

// ---------- HTML ----------
String ConfigManager::htmlRow(const String& k, const String& v) const {
    String eK = k; eK.replace("&", "&amp;"); eK.replace("<", "&lt;"); eK.replace(">", "&gt;");
    String eV = v; eV.replace("&", "&amp;"); eV.replace("<", "&lt;"); eV.replace(">", "&gt;");
    String s;
    s += "<tr><td><code>" + eK + "</code></td><td><input name='v_" + eK + "' value='" + eV + "' style='width:100%'></td>";
    s += "<td><form method='post' action='/delete' onsubmit='return confirm(\"Slette " + eK + "?\");'><input type='hidden' name='key' value='" + eK + "'><button>❌</button></form></td></tr>";
    return s;
}

String ConfigManager::htmlIndex() const {
    vector<String> keys;
    loadKeyList(keys);

    String rows;
    for (const auto& k : keys) {
        if (k == KEYLIST_KEY) continue;
        rows += htmlRow(k, const_cast<Preferences&>(_pref).getString(k.c_str(), ""));
    }

    String p = _pwd.length() >= 8 ? "(password protection)" : "(open)";
    String html =
        "<!doctype html><html><head><meta name=viewport content='width=device-width, initial-scale=1'>"
        "<title>Config Admin</title>"
        "<style>body{font-family:system-ui;margin:20px;} table{width:100%;border-collapse:collapse} td,th{border:1px solid #ddd;padding:6px;} code{background:#f3f3f3;padding:2px 4px;border-radius:4px}</style>"
        "</head><body>"
        "<h2>Config Admin – " + _ssid + " " + p + "</h2>"
        "<p>Wi‑Fi will turn off automatically shortly after startup.</p>"
        "<form method='post' action='/save'>"
        "<table><tr><th>Key</th><th>Value</th><th></th></tr>" +
        rows +
        "<tr><td><input name='new_key' placeholder='new_key'></td>"
        "<td><input name='new_val' placeholder='value' style='width:100%'></td>"
        "<td></td></tr>"
        "</table><br>"
        "<button>💾 Save changes</button> "
        "<a href='/export'>📦 Export</a>"
        "</form>"
        "<hr>"
        "<h3>Import</h3>"
        "<form method='post' action='/import'><textarea name='blob' rows=6 style='width:100%' placeholder='key=value per line'></textarea><br><button>⬆️ Import</button></form>"
        "</body></html>";
    return html;
}

void ConfigManager::handleIndex() {
    _server->send(200, "text/html", htmlIndex());
}

void ConfigManager::handleSave() {
    vector<String> keys;
    loadKeyList(keys);
    for (const auto& k : keys) {
        if (k == KEYLIST_KEY) continue;
        String formName = "v_" + k;
        if (_server->hasArg(formName)) {
            String val = _server->arg(formName);
            setString(k, val);
        }
    }
    if (_server->hasArg("new_key") && _server->hasArg("new_val")) {
        String nk = _server->arg("new_key");
        String nv = _server->arg("new_val");
        nk.trim();
        if (nk.length()) setString(nk, nv);
    }
    _server->sendHeader("Location", "/");
    _server->send(303);
}

void ConfigManager::handleDelete() {
    if (_server->hasArg("key")) {
        String k = _server->arg("key");
        remove(k);
    }
    _server->sendHeader("Location", "/");
    _server->send(303);
}

void ConfigManager::handleExport() {
    vector<String> keys;
    loadKeyList(keys);
    String out;
    for (const auto& k : keys) {
        if (k == KEYLIST_KEY) continue;
        String v = const_cast<Preferences&>(_pref).getString(k.c_str(), "");
        out += k + "=" + v + "\n";
    }
    _server->send(200, "text/plain", out);
}

void ConfigManager::handleImport() {
    if (_server->hasArg("blob")) {
        String blob = _server->arg("blob");
        int start = 0;
        while (true) {
            int nl = blob.indexOf('\n', start);
            String line = (nl < 0) ? blob.substring(start) : blob.substring(start, nl);
            line.trim();
            if (line.length()) {
                int eq = line.indexOf('=');
                if (eq > 0) {
                    String k = line.substring(0, eq);
                    String v = line.substring(eq + 1);
                    setString(k, v);
                }
            }
            if (nl < 0) break;
            start = nl + 1;
        }
    }
    _server->sendHeader("Location", "/");
    _server->send(303);
}

void ConfigManager::handleNotFound() {
    _server->send(404, "text/plain", "Not found");
}
