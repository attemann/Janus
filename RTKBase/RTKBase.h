#pragma once

int selectedRTCM = -1;  // Index of selected RTCM message
String inputLine = "";  // Buffer for brukerinndata

extern bool showGngga;
extern bool showFix;  
extern bool showRTCMenu;

extern GNSSModule gnss;  // GNSS serial port

void showMenu() {
    Serial.println("--- GNSS Console Commands ---");

    Serial.println("  ?       - Show help");
    Serial.println("  f       - Toggle fix output");
    Serial.println("  g       - Toggle gngga output");
    Serial.println("  r       - Show RTCM menu");
    Serial.println("  1-9     - Select a RTCM");

    Serial.println("  reset   - Reset the ESP32");
    Serial.println("  any other command");
    Serial.println("Type your command and press Enter:");
}

void handleConsoleCommand(String cmd) {
    cmd.trim();

    if (cmd == "f") {
        showFix = !showFix;
    }
    else if (cmd == "g") {
        showGngga = !showGngga;
    }
    else if (cmd == "r") {
        gnss.printRTCMConfig();
        Serial.println("Select 1-9 to choose an RTCM message.");
    }
    else if (cmd == "S") {
        gnss.sendConfiguredRTCMs();
        Serial.println("RTCM configuration sent to GNSS.");
    }
    else if (cmd.length() == 1 && isDigit(cmd[0])) {
        int idx = cmd.toInt() - 1;
        if (idx >= 0 && idx < gnss.getRTCMCount()) {
            selectedRTCM = idx;
            const auto& msg = gnss.getRTCM(idx);
            Serial.printf("Selected %s (%.2f Hz, %s)\n",
                msg.name, msg.frequencyHz, msg.enabled ? "ENABLED" : "DISABLED");
            Serial.println("  e        - toggle enable/disable");
            Serial.println("  =<Hz>    - change frequency (e.g., =0.5)");
        }
        else {
            Serial.println("Invalid RTCM index.");
        }
    }
    else if (cmd == "e" && selectedRTCM >= 0) {
        gnss.toggleRTCM(selectedRTCM);
        const auto& msg = gnss.getRTCM(selectedRTCM);
        Serial.printf("Toggled %s -> %s\n", msg.name, msg.enabled ? "ENABLED" : "DISABLED");
    }
    else if (cmd.startsWith("=") && selectedRTCM >= 0) {
        float freq = cmd.substring(1).toFloat();
        gnss.setRTCMFrequency(selectedRTCM, freq);
        const auto& msg = gnss.getRTCM(selectedRTCM);
        Serial.printf("Set %s frequency to %.2f Hz\n", msg.name, freq);
    }
    else if (cmd == "?") {
        showMenu();
    }
    else if (cmd.length() > 0) {
        gnss.sendCommand(cmd);
        Serial.print("> Sent to GNSS: ");
        Serial.println(cmd);
    }
}
void readConsole() {
    while (Serial.available()) {
        char ch = Serial.read();

        if (ch == '\r') continue;  // Ignorer carriage return
        if (ch == '\n') {
            handleConsoleCommand(inputLine);
            inputLine = "";
        }
        else {
            inputLine += ch;
        }

        // Sikkerhetsbuffer
        if (inputLine.length() > 80) inputLine = "";
    }
}