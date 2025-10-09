// SoundTask.cpp
#include "SoundTask.h"

bool SoundTask::begin(float gain,
    uint8_t bclk, uint8_t lrclk, uint8_t din,
    UBaseType_t taskPrio,
    uint32_t stackWords,
    BaseType_t core)
{
    if (_q) return true; // already started

    // 1) Init audio (mirrors your DecimalSpeaker::begin)
    _speaker.begin(gain, bclk, lrclk, din);

    // 2) Create queue
    _q = xQueueCreate(QUEUE_SIZE, sizeof(SoundCommand));
    if (!_q) {
        Serial.println("❌ SoundTask: failed to create queue");
        return false;
    }

    // 3) Start task (pinned to core by default for glitch-free audio)
    BaseType_t ok = xTaskCreatePinnedToCore(
        taskRunner,
        "SoundTask",
        stackWords,
        this,
        taskPrio,
        &_task,
        core
    );
    if (ok != pdPASS) {
        Serial.println("❌ SoundTask: failed to start task");
        vQueueDelete(_q);
        _q = nullptr;
        return false;
    }

    Serial.printf("✅ SoundTask started (core %d, prio %u)\n", (int)core, (unsigned)taskPrio);
    return true;
}

bool SoundTask::send(const SoundCommand& cmd, TickType_t timeoutTicks) {
    if (!_q) return false;
    return xQueueSend(_q, &cmd, timeoutTicks) == pdTRUE;
}

size_t SoundTask::queued() const {
    if (!_q) return 0;
    return uxQueueMessagesWaiting(_q);
}

void SoundTask::taskRunner(void* arg) {
    auto* self = static_cast<SoundTask*>(arg);
    SoundCommand cmd;
    for (;;) {
        if (xQueueReceive(self->_q, &cmd, portMAX_DELAY) == pdTRUE) {
            self->handle(cmd);
        }
    }
}

void SoundTask::handle(const SoundCommand& cmd) {
    switch (cmd.type) {
    case SoundCmdType::WAV:
        if (cmd.path) _speaker.playWavFile(cmd.path);
        break;
    case SoundCmdType::NUMBER:     _speaker.playNumberFile(cmd.intValue); break;
    case SoundCmdType::INT:        _speaker.speakInt(cmd.intValue); break;
    case SoundCmdType::TIME:       _speaker.speakTime(cmd.floatValue); break;
    case SoundCmdType::ERROR:      _speaker.speakError(cmd.intValue); break;
    case SoundCmdType::INFO:       _speaker.speakInfo(cmd.intValue); break;
    case SoundCmdType::FIX:        _speaker.speakFix(cmd.intValue); break;
    case SoundCmdType::STARTING:   _speaker.speakStarting(); break;
    case SoundCmdType::GETTINGFIX: _speaker.speakGettingFix(); break;
    case SoundCmdType::SURVEY:     _speaker.speakSurvey(); break;
    case SoundCmdType::OPERATING:  _speaker.speakOperating(); break;
    }
}