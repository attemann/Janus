// SoundTask.h
#pragma once
#include <Arduino.h>
#include "Sound.h"   // your DecimalSpeaker
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

enum class SoundCmdType {
    WAV, NUMBER, INT, TIME,
    ERROR, INFO, FIX,
    STARTING, GETTINGFIX, SURVEY, OPERATING, GPS
};

struct SoundCommand {
    SoundCmdType type;
    int          intValue = 0;
    float        floatValue = 0.0f;
    const char* path = nullptr; // must outlive send() or copy into a buffer
};

class SoundTask {
public:
    // Mirrors DecimalSpeaker::begin signature (+ optional core/prio/stack)
    bool begin(float gain,
        uint8_t bclk = 26, uint8_t lrclk = 22, uint8_t din = 25,
        UBaseType_t taskPrio = 1,
        uint32_t stackWords = 4096,
        BaseType_t core = 1);

    bool send(const SoundCommand& cmd, TickType_t timeoutTicks = 0);
    size_t queued() const;

private:
    static void taskRunner(void* arg);
    void handle(const SoundCommand& cmd);

    static constexpr size_t QUEUE_SIZE = 12;

    DecimalSpeaker _speaker;
    QueueHandle_t  _q = nullptr;
    TaskHandle_t   _task = nullptr;
};
