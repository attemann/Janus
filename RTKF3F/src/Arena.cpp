#include <Arduino.h>
#include <math.h>
#include <RTKF3F.h>
#include "Arena.h"

Arena::Arena() {
    _name[0] = '\0';  // More explicit null terminator
    initDetection();  // Initialize geometry with defaults
}

void Arena::setName(const char* n) {
    if (!n) return;  // Safety check for nullptr
    strncpy(_name, n, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
}

const char* Arena::getName() const {
    return _name;
}

void Arena::setCourseLength(int l) {  // Remove default parameter from implementation
    // Add validation
    if (l < 10 || l > 1000) {
        Serial.println("Warning: Course length out of range (10-1000m)");
        return;
    }
    _courseLength = l;
    initDetection();
}

int Arena::getCourseLength() const {
    return _courseLength;
}

void Arena::setCourseDirection(int dir) {
    // Normalize to 0-359 range
    _courseDir = dir % 360;
    if (_courseDir < 0) _courseDir += 360;
    initDetection();
}

int Arena::getCourseDirection() const {
    return _courseDir;
}

void Arena::incrementCourseDirection(int incr) {
    _courseDir = (_courseDir + incr) % 360;
    if (_courseDir < 0) _courseDir += 360;
    initDetection();
}

void Arena::setPilotOffsetNED(int north, int east, int down) {
    _offsetNorth = north;
    _offsetEast = east;
    _offsetDown = down;
    // Note: Currently offsets aren't used in initDetection()
    // If they should affect geometry, add them to the calculation
}

void Arena::getPilotOffsetNED(int& north, int& east, int& down) const {
    north = _offsetNorth;
    east = _offsetEast;
    down = _offsetDown;
}

void Arena::setABaseLeft(bool left) {
    _aBaseOnLeft = left;
    initDetection();
}

bool Arena::getABaseLeft() const {
    return _aBaseOnLeft;
}

void Arena::toggleABaseSide() {
    _aBaseOnLeft = !_aBaseOnLeft;
    initDetection();
}

// Encode
void Arena::encodeArenaSettings(uint8_t* buf, bool abaseLeft, uint16_t length, uint16_t courseDirection) {
    if (!buf) return;  // Safety check

    buf[0] = static_cast<uint8_t>(MessageType::MSG_ARENA_SETTINGS);
    buf[1] = abaseLeft ? 1 : 0;  // 1=left, 0=right (more intuitive)
    buf[2] = (length >> 8) & 0xFF;
    buf[3] = length & 0xFF;
    buf[4] = (courseDirection >> 8) & 0xFF;
    buf[5] = courseDirection & 0xFF;
    // buf[6..] reserved for future expansion (name, laps, etc)
}

// Decode
void Arena::decodeArenaSettings(const uint8_t* buf) {
    if (!buf) return;  // Safety check

    // Expected: buf[0] == MSG_ARENA_SETTINGS (caller should verify)
    _aBaseOnLeft = (buf[1] == 1);
    _courseLength = (static_cast<uint16_t>(buf[2]) << 8) | buf[3];
    _courseDir = (static_cast<uint16_t>(buf[4]) << 8) | buf[5];

    // Validate decoded values
    if (_courseLength < 10 || _courseLength > 1000) {
        Serial.println("Warning: Decoded invalid course length");
        _courseLength = 100;  // Reset to default
    }
    if (_courseDir >= 360) {
        _courseDir = _courseDir % 360;
    }

    // Call initDetection() only once after all updates
    initDetection();
}

void Arena::initDetection() {
    // Convert course direction to radians
    // Course direction: 0° = north, increases clockwise
    float rad = radians(90.0f - _courseDir);

    // Unit vector in slope direction (along the course line)
    float slope_dx = cos(rad);
    float slope_dy = sin(rad);

    // Normal vector (perpendicular to slope, pointing to A-base side)
    _slope_nx = -slope_dy;
    _slope_ny = slope_dx;

    // Flip normal if A-base is on the right
    if (!_aBaseOnLeft) {
        _slope_nx = -_slope_nx;
        _slope_ny = -_slope_ny;
    }

    // Calculate base positions
    // Bases are placed along the normal, centered on pilot position (0,0)
    float halfLength = _courseLength / 2.0f;

    _A_x = _slope_nx * -halfLength;
    _A_y = _slope_ny * -halfLength;
    _B_x = _slope_nx * halfLength;
    _B_y = _slope_ny * halfLength;
}

// Getters for calculated geometry
float Arena::getAx() const { return _A_x; }
float Arena::getAy() const { return _A_y; }
float Arena::getBx() const { return _B_x; }
float Arena::getBy() const { return _B_y; }
float Arena::getSlopeNx() const { return _slope_nx; }
float Arena::getSlopeNy() const { return _slope_ny; }