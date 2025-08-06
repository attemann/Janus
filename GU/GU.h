#pragma once
String inputLine = "";  // Buffer for brukerinndata

extern HardwareSerial SerialGNSS;  // GNSS serial port
extern bool showFix;
extern bool showRTCM;
extern bool showGngga;

void showMenu() {
    Serial.println("--- GNSS Console Commands ---");

    Serial.println("  ?       - Show help");
    Serial.println("  g       - Enable gngga output");
    Serial.println("  f       - Enable fix output");
    Serial.println("  r       - Log RTCM messages");
    Serial.println("  reset   - Reset the ESP32");
    Serial.println("  any other command");
	Serial.println("Type your command and press Enter:");
}

void handleConsoleCommand(String cmd) {
    cmd.trim();  // Fjern mellomrom og linjeskift

    if (cmd == "f") {
		showFix = !showFix;  // Toggle fix output
    }
    else if (cmd == "r") {
        showRTCM = !showRTCM;  // Toggle RTCM output
    }
    else if (cmd == "g") {
        showGngga = !showGngga;  // Toggle RTCM output
    }
    else if (cmd == "reset") {
        Serial.println("Resetting ESP32...");
        ESP.restart();
    }
    else if (cmd == "?") {
		showMenu();
    }
    else if (cmd.length() > 0) {
        // Send ukjent kommando direkte til GNSS
        SerialGNSS.println(cmd);
        Serial.print("? Sent to GNSS: ");
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

