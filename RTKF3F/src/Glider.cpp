#include <RTKF3F.h>
#include "Glider.h"
#include "Arena.h"  

Glider::Glider() {}

void Glider::setId(uint8_t id) { _id = id; }
uint8_t Glider::getId() const { return _id; }

void Glider::setLaunched(bool state) { _launched = state; if (state && _launchCallback) _launchCallback(*this); }
bool Glider::isLaunched() const { return _launched; }

void Glider::setLanded(bool state) { _landed = state; if (state && _landCallback) _landCallback(*this); }
bool Glider::isLanded() const { return _landed; }

void Glider::setArena(const Arena* arena) { _arena = arena; }
const Arena* Glider::getArena() const { return _arena; }

void Glider::onCrossA(void (*cb)(Glider&)) { _crossACallback = cb; }
void Glider::onCrossB(void (*cb)(Glider&)) { _crossBCallback = cb; }
void Glider::onLaunch(void (*cb)(Glider&)) { _launchCallback = cb; }
void Glider::onLand(void (*cb)(Glider&)) { _landCallback = cb; }

void Glider::setInitialPosition(int32_t n, int32_t e, int32_t d) {
    _initialN = n;
    _initialE = e;
    _initialD = d;
}

void Glider::getInitialPosition(int32_t& n, int32_t& e, int32_t& d)  {
    n = _initialN; e = _initialE; d = _initialD;
}

bool Glider::checkCrossing(const GNSSModule::GNSSFix& fix, MessageType& event) {
    if (!_arena) return false;  // Safety

    static bool wasBetweenBases = false;

    float px = fix.adjEast;
    float py = fix.adjNorth;

    // Query Arena for all geometry
    float A_x = _arena->getAx();
    float A_y = _arena->getAy();
    float B_x = _arena->getBx();
    float B_y = _arena->getBy();
    float slope_nx = _arena->getSlopeNx();
    float slope_ny = _arena->getSlopeNy();

    // Project position onto normal at A and B
    float dA = (px - A_x) * slope_nx + (py - A_y) * slope_ny;
    float dB = (px - B_x) * slope_nx + (py - B_y) * slope_ny;

    bool between = (dA > 0 && dB < 0);

    if (!wasBetweenBases && between) {
        wasBetweenBases = true;
        event = (dA <= 0) ? MessageType::MSG_EVT_CROSS_A_IN : MessageType::MSG_EVT_CROSS_B_IN;
        return true;
    }

    if (wasBetweenBases && !between) {
        wasBetweenBases = false;
        event = (dA <= 0) ? MessageType::MSG_EVT_CROSS_A_OUT : MessageType::MSG_EVT_CROSS_B_OUT;
        return true;
    }

    return false;
}
