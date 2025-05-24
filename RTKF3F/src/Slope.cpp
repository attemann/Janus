//Slope.cpp
#include "Const.h"
#include "Slope.h"

void Slope::setGliderId(int g) {
    glider=g;
}   

int Slope::getGliderId() const {
    return glider;
}

void Slope::incrementGliderId(int incr){ 
    glider += incr;
    if (glider < 1) {
        glider = MAX_GU_UNITS;
    } else if (glider > MAX_GU_UNITS) {
        glider = 1;
    }
}

void Slope::setSlopeAngle(int ang) {
    angle = ang;
}

int Slope::getSlopeAngle() const{
    return angle;
}

void Slope::incrementSlopeAngle(int incr){
    angle=angle+incr;
    if (angle > 360) angle -= 360;
    if (angle < 0)   angle += 360;
    //if (angle = 360) angle = 0;
}      

void Slope::setABaseLeft (bool left) {
    aBaseOnLeft = left;
} 

bool Slope::getABaseLeft() const {
    return aBaseOnLeft;
} 

void Slope::toggleABaseSide() {
    aBaseOnLeft=!aBaseOnLeft;
}

void Slope::setPilotOffsetNED(int north, int east, int down) {
    offsetNorth = north;
    offsetEast = east;
    offsetDown = down;
}
void Slope::getPilotOffsetNED(int &north, int &east, int &down) const {
    north = offsetNorth;
    east = offsetEast;
    down = offsetDown;
}