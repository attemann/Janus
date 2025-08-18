#pragma once

#include <stdint.h>
#include <RTKF3F.h>

class Arena;  // Forward declaration

// === Event codes (Bits 7–4 of Byte 1) ===
// Events sent from Glider Unit (GU) to Base Station (BS)
enum EventCode : uint8_t {
    EVT_NONE = 0x0,  // Reserved / no event
    EVT_CROSS_A_IN = 0x1,  // Cross A base into task
    EVT_CROSS_A_OUT = 0x2,  // Cross A base out of task
    EVT_CROSS_B_IN = 0x3,  // Cross B base into task
    EVT_CROSS_B_OUT = 0x4,  // Cross B base out of task
    EVT_SAFETY_IN_TO = 0x5,  // Cross into safety area (danger zone)
    EVT_SAFETY_OUT_OF = 0x6,  // Cross out of safety area (safe zone)
    EVT_AIRBORNE = 0x7,  // Glider airborne (takeoff)
    EVT_LANDED = 0x8,  // Glider landed
    EVT_ACK = 0x9,  // Acknowledgment of flight settings
};

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

    bool checkCrossing(const GNSSModule::GNSSFix& fix, EventCode& event);

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