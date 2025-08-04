##pragma once

#include "SPIFFS.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"

/*

Upload:
======
C:\mkspiffs\mkspiffs.exe -c data -b 4096 -p 256 -s 0x160000 spiffs.bin
python -m esptool --chip esp32 --port COM5 write-flash 0x290000 spiffs.bin

Usage:
======
DecimalSpeaker speaker;

void setup() {
    Serial.begin(115200);
    delay(500);

    speaker.begin();
}

void loop() {
    speaker.speak(0.1);
    delay(500);
    speaker.speak(0.12);
    delay(500);
    speaker.speak(09.00);  v
    delay(500);
    speaker.speak(34.29);
    delay(500);
    speaker.speak(45.01);
    delay(500);
    speaker.speak(100.29);
    delay(500);
}
*/

class DecimalSpeaker {
public:
    void begin() {
        if (!SPIFFS.begin(true)) {
            Serial.println("❌ Failed to mount SPIFFS");
            return;
        }
        Serial.println("✅ SPIFFS mounted");

        listSpiffsFiles();

        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        while (file) {
            Serial.printf("📂 %s (%d bytes)\n", file.name(), file.size());
            file = root.openNextFile();
        }
    }


    void speak(float value) {
        if (value < 0) return;
        if (value > 99.99f) value = 99.99f;

        int intPart = static_cast<int>(value);
        int fracPart = static_cast<int>((value - intPart) * 100 + 0.5f);

        // --- Heltallsdel ---
        if (intPart <= 20) {
            playNumberFile(intPart);
        }
        else {
            int tens = intPart / 10;
            int ones = intPart % 10;
            playNumberFile(tens * 10);
            if (ones > 0) playNumberFile(ones);
        }

        playWavFile("/point.wav");

        // --- Desimaldel ---
        if (fracPart < 10) {
            playNumberFile(0);            // "zero"
            playNumberFile(fracPart);     // 1-9
        }
        else if (fracPart <= 20) {
            playNumberFile(fracPart);     // 10-20
        }
        else {
            int tens = fracPart / 10;
            int ones = fracPart % 10;
            playNumberFile(tens * 10);    // 30, 40, ...
            if (ones > 0) playNumberFile(ones);
        }

        playWavFile("/Seconds.wav");
    }

private:
    void listSpiffsFiles() {
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        if (!file) Serial.println("⚠️ SPIFFS is empty!");
        while (file) {
            Serial.printf("🗂️ %s (%d bytes)\n", file.name(), file.size());
            file = root.openNextFile();
        }
    }

    bool playWavFile(const char* path) {
        if (!SPIFFS.exists(path)) {
            Serial.printf("❌ File does not exist: %s\n", path);
            return false;
        }

        Serial.printf("🎧 Playing: %s\n", path);

        AudioFileSourceSPIFFS* file = new AudioFileSourceSPIFFS(path);
        AudioOutputI2S* out = new AudioOutputI2S();
        out->SetPinout(26, 22, 25);   // BCLK, LRCLK, DIN
        out->SetGain(0.1);
        out->SetOutputModeMono(true); // Force mono

        AudioGeneratorWAV* wav = new AudioGeneratorWAV();
        if (!wav->begin(file, out)) {
            Serial.println("❌ AudioGeneratorWAV::begin: file not open");
            delete file;
            delete out;
            delete wav;
            return false;
        }

        while (wav->isRunning()) {
            if (!wav->loop()) break;
        }

        wav->stop();
        delete wav;
        delete file;
        delete out;

        //Serial.println("✅ Done");
        return true;
    }

    void playNumberFile(int number) {
        char path[20];
        snprintf(path, sizeof(path), "/%d.wav", number);
        playWavFile(path);
    }
};
