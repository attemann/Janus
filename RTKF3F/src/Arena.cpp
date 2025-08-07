#include <Arduino.h>
#include <math.h>
#include <RTKF3F.h>
#include "Arena.h"

Arena::Arena() {
    _name[0] = 0;
}

void Arena::setName(const char* n) {
    strncpy(_name, n, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = 0;
}

const char* Arena::getName() const {
    return _name;
}

void Arena::setCourseLength(int l=100) {
    _courseLength = l;
    initDetection();
}

int Arena::getCourseLength() const {
    return _courseLength;
}

void Arena::setCourseDirection(int dir) {
    _courseDir = dir;
    initDetection();
}

int Arena::getCourseDirection() const {
    return _courseDir;
}

void Arena::incrementCourseDirection(int incr) {
    _courseDir += incr;
    while (_courseDir >= 360) _courseDir -= 360;
    while (_courseDir < 0)    _courseDir += 360;
    initDetection();
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
    buf[0] = MSG_ARENA_SETTINGS;
    buf[1] = abaseLeft ? 0 : 1;  // 0=left, 1=right (just flip if you prefer opposite logic)
    buf[2] = (length >> 8) & 0xFF;   // High byte
    buf[3] = length & 0xFF;          // Low byte
    buf[4] = (courseDirection >> 8) & 0xFF;
    buf[5] = courseDirection & 0xFF;
    // buf[6..] for utvidelse (f.eks. navn, antall runder, etc)
}

// Decode
void Arena::decodeArenaSettings(const uint8_t* buf) {
    // Forvent at buf[0]==MSG_ARENA_SETTINGS!
    _aBaseOnLeft = (buf[1] == 0);    // 0=left
    _courseLength = (static_cast<uint16_t>(buf[2]) << 8) | buf[3];
    _courseDir = (static_cast<uint16_t>(buf[4]) << 8) | buf[5];
}

void Arena::initDetection() {
    float rad = radians(90.0f - _courseDir);  // 0° = north, øker med klokka

    // Unit vector in slope direction
    float slope_dx = cos(rad);
    float slope_dy = sin(rad);

    // Normal vector (perpendicular to slope direction)
    _slope_nx = -slope_dy;
    _slope_ny = slope_dx;

    // Flip direction if A base is on the left
    if (!_aBaseOnLeft) {
        _slope_nx *= -1;
        _slope_ny *= -1;
    }

    // Place A and B bases relative to pilot location (0,0)
    _A_x = _slope_nx * -(_courseLength / 2);
    _A_y = _slope_ny * -(_courseLength / 2);
    _B_x = _slope_nx * (_courseLength / 2);
    _B_y = _slope_ny * (_courseLength / 2);
}

float Arena::getAx() const { return _A_x; }
float Arena::getAy() const { return _A_y; }
float Arena::getBx() const { return _B_x; }
float Arena::getBy() const { return _B_y; }
float Arena::getSlopeNx() const { return _slope_nx; }
float Arena::getSlopeNy() const { return _slope_ny; }