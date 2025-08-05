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

#pragma once

#include "SPIFFS.h"
#include "AudioFileSourceSPIFFS.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"

class DecimalSpeaker {
public:
    void begin(float gain, uint8_t bclk = 26, uint8_t lrclk = 22, uint8_t din = 25) {
        if (!SPIFFS.begin(true)) {
            Serial.println("❌ Failed to mount SPIFFS");
            return;
        }
        Serial.println("✅ SPIFFS mounted");
        listSpiffsFiles();

        _audioOut = new AudioOutputI2S();
        _audioOut->SetPinout(bclk, lrclk, din);
        _audioOut->SetGain(gain);
        _audioOut->SetOutputModeMono(true); // Force mono
    }

    bool playWavFile(const char* path) {
        if (!SPIFFS.exists(path)) {
            Serial.printf("❌ File does not exist: %s\n", path);
            return false;
        }

        Serial.printf("🎧 Playing: %s\n", path);

        AudioFileSourceSPIFFS* file = new AudioFileSourceSPIFFS(path);
        AudioGeneratorWAV* wav = new AudioGeneratorWAV();

        if (!wav->begin(file, _audioOut)) {
            Serial.println("❌ AudioGeneratorWAV::begin: file not open");
            delete file;
            delete wav;
            return false;
        }

        while (wav->isRunning()) {
            if (!wav->loop()) break;
        }

        wav->stop();
        delete wav;
        delete file;
        return true;
    }

    void playNumberFile(int number) {
        char path[20];
        snprintf(path, sizeof(path), "/%d.wav", number);
        playWavFile(path);
    }

    void speakError(int errorCode) {
        if (errorCode < 0 || errorCode > 99) return;

        playWavFile("/error.wav");
        if (errorCode < 10) {
            playNumberFile(0);
        }
        playNumberFile(errorCode);
	}

    void speakTime(float value) {
        if (value < 0) return;
        if (value > 99.99f) value = 99.99f;

        int intPart = static_cast<int>(value);
        int fracPart = static_cast<int>((value - intPart) * 100 + 0.5f);

        playWavFile("/time.wav");

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

        if (fracPart < 10) {
            playNumberFile(0);
            playNumberFile(fracPart);
        }
        else if (fracPart <= 20) {
            playNumberFile(fracPart);
        }
        else {
            int tens = fracPart / 10;
            int ones = fracPart % 10;
            playNumberFile(tens * 10);
            if (ones > 0) playNumberFile(ones);
        }

        playWavFile("/seconds.wav");
    }

private:
    AudioOutputI2S* _audioOut = nullptr;

    void listSpiffsFiles() {
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        if (!file) Serial.println("⚠️ SPIFFS is empty!");
        while (file) {
            Serial.printf("🗂️ %s (%d bytes)\n", file.name(), file.size());
            file = root.openNextFile();
        }
    }

};
