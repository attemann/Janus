// Sound.h
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

#include <Arduino.h>
#include <SPIFFS.h>
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
        //listSpiffsFiles();

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

        playWavFile("/error.wav");
        if (errorCode < 10) {
            playNumberFile(0);
        }
        playNumberFile(errorCode);
	}

    void speakInfo(int infoCode) {

        playWavFile("/information.wav");
        speakInt(infoCode);
    }

    void speakFix(int fix) {
        playWavFile("/fixtype.wav");
        playNumberFile(fix);
	}

    void speakStarting() {
        playWavFile("/starting.wav");
	}

    void speakGettingFix() {
        playWavFile("/getting_fix.wav");
    }

    void speakSurvey() {
        playWavFile("/survey.wav");
    }

    void speakOperating() {
        playWavFile("/operating.wav");
    }

    void speakInt(int value) {
        if (value < 0) return;
        if (value > 9999) value = 9999;

        // Thousands
        if (value >= 1000) {
            int thousands = value / 1000;
            playNumberFile(thousands);
            playWavFile("/thousand.wav"); // "thousand"
            value %= 1000;
        }

        // Hundreds
        if (value >= 100) {
            int hundreds = value / 100;
            playNumberFile(hundreds);
            playNumberFile(100); // "hundred"
            value %= 100;
        }

        // 0–99: use optimal grouping (no redundant "zero")
        if (value > 0) {
            if (value <= 20) {
                playNumberFile(value);
            }
            else {
                int tens = value / 10 * 10;
                int ones = value % 10;
                playNumberFile(tens);
                if (ones > 0) playNumberFile(ones);
            }
        }
        else if (value == 0 && (value == 0 || (value < 100 && value > 0))) {
            // For e.g., "one thousand zero"
            // Only speak "zero" if entire value was zero, or after "thousand" or "hundred" as needed
            playNumberFile(0);
        }
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
