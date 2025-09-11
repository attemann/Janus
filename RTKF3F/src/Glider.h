#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <RTKF3F.h>

class Arena;  // Forward declaration

class Glider {
public:
    Glider();

    void setId(uint8_t id);
    uint8_t getId() const;

    void setLaunched(bool state);
    bool isLaunched() const;

    void setLanded(bool state);
    bool isLanded() const;

    void setArena(const Arena* arena);
    const Arena* getArena() const;

    // Event callbacks:
    void onCrossA(void (*cb)(Glider&));
    void onCrossB(void (*cb)(Glider&));
    void onLaunch(void (*cb)(Glider&));
    void onLand(void (*cb)(Glider&));

    void setInitialPosition(int32_t n, int32_t e, int32_t d);
    void getInitialPosition(int32_t& n, int32_t& e, int32_t& d);

    bool checkCrossing(const GNSSFix& fix, MessageType& event);

private:
    uint8_t _id = 0;
    int32_t _initialN = 0, _initialE = 0, _initialD = 0;
    bool _launched = false, _landed = true;

    const Arena* _arena = nullptr;

    // Callbacks:
    void (*_crossACallback)(Glider&) = nullptr;
    void (*_crossBCallback)(Glider&) = nullptr;
    void (*_launchCallback)(Glider&) = nullptr;
    void (*_landCallback)(Glider&) = nullptr;

    // For crossing detection:
    bool _lastWasA = false;
    bool _lastWasB = false;
};