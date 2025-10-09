#pragma once

class Arena {
public:
    Arena();

    // Name management
    void setName(const char* n);
    const char* getName() const;

    // Course configuration
    void setCourseLength(int l);  // meters, valid range: 10-1000
    int getCourseLength() const;

    void setCourseDirection(int dir);  // degrees (0=north, 90=east, clockwise)
    int getCourseDirection() const;
    void incrementCourseDirection(int incr);

    // Pilot offset (NED coordinates relative to course origin)
    void setPilotOffsetNED(int north, int east, int down);
    void getPilotOffsetNED(int& north, int& east, int& down) const;

    // A-base side configuration
    void setABaseLeft(bool left);
    bool getABaseLeft() const;
    void toggleABaseSide();

    // Message encoding/decoding for radio transmission
    void encodeArenaSettings(uint8_t* buf, bool abaseLeft, uint16_t length, uint16_t courseDirection);
    void decodeArenaSettings(const uint8_t* buf);

    // Calculated geometry getters
    float getAx() const;
    float getAy() const;
    float getBx() const;
    float getBy() const;
    float getSlopeNx() const;  // Normal vector X component
    float getSlopeNy() const;  // Normal vector Y component

private:
    // Configuration
    char _name[32];
    int _courseLength = 100;     // meters (default 100)
    int _courseDir = 0;          // degrees (0=north, 90=east, clockwise)
    int _offsetNorth = 0;        // meters
    int _offsetEast = 0;         // meters
    int _offsetDown = 0;         // meters
    bool _aBaseOnLeft = false;   // true = A-base on left side

    // Calculated geometry (updated by initDetection)
    float _A_x, _A_y;            // A-base position in local coordinates
    float _B_x, _B_y;            // B-base position in local coordinates
    float _slope_nx, _slope_ny;  // Normal vector (perpendicular to course line)

    // Recalculates geometry when configuration changes
    void initDetection();
};