#pragma once

#include <Arduino.h>

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <vector>
using std::vector;

struct AdminOpts {
    String ssid;
    String password;
    IPAddress apIP;
    uint8_t channel = 1;
    uint32_t windowMs = 0; // varighet i millisekunder (f.eks. 120000 for 2 minutter)
};

class ConfigManager {
public:
    ConfigManager(const char* nvsNamespace);

    // NVS (flash-lagring)
    bool begin();
    bool clearAll();
    bool exists(const String& key) const;

    bool setString(const String& key, const String& value);
    String getString(const String& key, const String& def = "") const;

    bool setInt(const String& key, int32_t value);
    int32_t getInt(const String& key, int32_t def = 0) const;

    bool setBool(const String& key, bool value);
    bool getBool(const String& key, bool def = false) const;

    bool setFloat(const String& key, float value);
    float getFloat(const String& key, float def = 0.0f) const;

    bool remove(const String& key);
    void getKeys(vector<String>& out) const;

    // Admin-AP
    void startAdminWindow(const AdminOpts& opts);
    void stopAdmin();
    void loop();

private:
    String _ns;
    Preferences _pref;

    // nøkkelliste
    void loadKeyList(vector<String>& keys) const;
    bool saveKeyList(const vector<String>& keys) const;
    bool ensureKeyInList(const String& key);
    bool removeKeyFromList(const String& key);

    // Admin-WiFi
    WebServer* _server = nullptr;
    bool _adminActive = false;
    uint32_t _adminEndMs = 0;

    String _ssid;
    String _pwd;
    IPAddress _apIP;
    uint8_t _apChannel;

    // HTTP-server
    void setupHttp();
    void handleCaptive();
    void handleIndex();
    void handleSave();
    void handleDelete();
    void handleExport();
    void handleImport();
    void handleNotFound();

    // HTML
    String htmlRow(const String& key, const String& value) const;
    String htmlIndex() const;
};
