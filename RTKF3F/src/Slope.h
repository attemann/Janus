//slope.h
#pragma once

#ifndef SLOPE_H
#define SLOPE_H

class Slope {   
private:
    int glider=1;
    int angle=180;
    bool aBaseOnLeft = false;

    int offsetNorth=0;
    int offsetEast=0;
    int offsetDown=0;  

public:
    void setGliderId(int g);
    int getGliderId() const;
    void incrementGliderId(int incr);
    void setSlopeAngle(int ang);
    int getSlopeAngle() const;
    void incrementSlopeAngle(int incr);
    void setABaseLeft (bool left);
    bool getABaseLeft() const;
    void toggleABaseSide();
    void setPilotOffsetNED(int north, int east, int down);
    void getPilotOffsetNED(int &north, int &east, int &down) const;
};

#endif