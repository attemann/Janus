//Airborne.h
#pragma once

#ifndef AIRBORNE_DETECTOR_H
#define AIRBORNE_DETECTOR_H

    #include <Arduino.h>
    #include <RTKF3F.h>

    class AirborneDetector {
    public:
        AirborneDetector();

        void begin(unsigned long intervalMs = 100);
        void update(float north, float east);
        void setThresholds(float airborneDist, float landedDist);
        bool isCurrentlyAirborne() const;

    private:
        struct Position {
            float north;
            float east;
        };

        Position posBuffer[DETECTOR_BUFFER_SIZE];
        int bufferIndex;
        bool airborne;
        float thresholdAirborne;
        float thresholdLanded;
        unsigned long lastSampleTime;
        unsigned long sampleInterval;
    };

#endif