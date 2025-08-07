#pragma once

class Arena {
public:
    Arena();

    void setName(const char* n);
    const char* getName() const;

    void setCourseLength(int l);
    int getCourseLength() const;

    void setCourseDirection(int dir);
    int getCourseDirection() const;
    void incrementCourseDirection(int incr);

    void setPilotOffsetNED(int north, int east, int down);
    void getPilotOffsetNED(int& north, int& east, int& down) const;

    void setABaseLeft(bool left);
    bool getABaseLeft() const;
    void toggleABaseSide();

    void encodeArenaSettings(uint8_t* buf, bool abaseLeft, uint16_t length, uint16_t courseDirection);
    void decodeArenaSettings(const uint8_t* buf);

    float getAx() const ;
    float getAy() const;
    float getBx() const;
    float getBy() const;
    float getSlopeNx() const;
    float getSlopeNy() const;

private:
    char _name[32];
    int _courseLength = 100;  // meters (default 100)
    int _courseDir = 0;       // degrees (0=north, 90=east)

    int _offsetNorth = 0;
    int _offsetEast = 0;
    int _offsetDown = 0;
    bool _aBaseOnLeft = false;

    float _A_x, _A_y;
    float _B_x, _B_y;
	float _slope_nx, _slope_ny;

    void initDetection();
};