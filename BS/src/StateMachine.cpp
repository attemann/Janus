// StateMachine.cpp
#include <Arduino.h>
#include <cmath>
#include <LiquidCrystal_I2C.h>
#include <RTKF3F.h>
#include "StateMachine.h"

extern LiquidCrystal_I2C lcd;
extern Slope slope;

void directionAndDistance(int n, int e, int d, int &angle, float &dist) {
    // Beregn horisontal avstand i N/E-plan
    dist = sqrt(n * n + e * e);

    // Beregn vinkel fra nord (0° = nord, 90° = øst, osv.)
    float angleRad = atan2(e, n);
    float angleDeg = angleRad * 180.0 / M_PI;

    // Normaliser til 0–359°
    if (angleDeg < 0) angleDeg += 360.0;

    angle = (int)(round(angleDeg));
}

String formatTimeMSS(unsigned long seconds) {
    unsigned int minutes = seconds / 60;
    unsigned int secs = seconds % 60;
    char buffer[12];  
    snprintf(buffer, sizeof(buffer), "%02u:%02u", minutes, secs);
    return String(buffer);
}

String formatTimeSSS(unsigned long ms) {
    unsigned seconds = ms / 1000;
    unsigned fraction = (ms % 1000) / 10;  // 2 desimaler
    char buffer[12];  
    snprintf(buffer, sizeof(buffer), "%03u.%02u", seconds, fraction);
    return String(buffer);
}

void padRight(char* buffer, size_t totalLength) {
    size_t len = strlen(buffer);
    if (len < totalLength) {
        memset(buffer + len, ' ', totalLength - len);  // fill with spaces
        buffer[totalLength] = '\0';                    // null-terminate
    }
    else if (len > totalLength) {
        buffer[totalLength] = '\0';  // truncate if too long (just in case)
    }
}

void StateMachine::begin() {
    bsState = BS_WAITING;
    bsTaskState = TASK_UNKNOWN;
    startTimestamp = 0;
    finishTimestamp = 0;
}

void StateMachine::handleEvent(EventCode event) {
    switch (bsTaskState) {
        case TASK_UNKNOWN:
            if (event == EVT_CROSS_A_OUT)
                bsTaskState = TASK_OUTSIDE_A;
            break;

        case TASK_OUTSIDE_A:
            if (event == EVT_CROSS_A_IN)
                bsTaskState = TASK_STARTED;
                startTimestamp = millis();
            break;

        case TASK_STARTED:
            if (event == EVT_CROSS_B_OUT)
                bsTaskState = TASK_TURN1;
            break;

        case TASK_TURN1:
            if (event == EVT_CROSS_A_OUT)
                bsTaskState = TASK_TURN2;
            break;

        case TASK_TURN2:
            if (event == EVT_CROSS_B_OUT)
                bsTaskState = TASK_TURN3;
            break;

        case TASK_TURN3:
            if (event == EVT_CROSS_A_OUT)
                bsTaskState = TASK_TURN4;
            break;

        case TASK_TURN4:
            if (event == EVT_CROSS_B_OUT)
                bsTaskState = TASK_TURN5;
            break;

        case TASK_TURN5:
            if (event == EVT_CROSS_A_OUT)
                bsTaskState = TASK_TURN6;
            break;

        case TASK_TURN6:
            if (event == EVT_CROSS_B_OUT)
                bsTaskState = TASK_TURN7;
            break;

        case TASK_TURN7:
            if (event == EVT_CROSS_A_OUT)
                bsTaskState = TASK_TURN8;
            break;

        case TASK_TURN8:
            if (event == EVT_CROSS_B_OUT)
                bsTaskState = TASK_TURN9;
            break;

        case TASK_TURN9:
            if (event == EVT_CROSS_A_OUT) {
                bsTaskState = TASK_FINISHED;
                finishTimestamp = millis();
            }
            break;

        case TASK_FINISHED:
            if (millis() - finishTimestamp > 5000) {
                bsTaskState = TASK_UNKNOWN;
            }
            break;

        default:
            break;
    }
}

void StateMachine::handleButton(int buttonId) {

    //Serial.print("Handling button ");
    //switch(buttonId) {
    //  case BTN_MNU: Serial.print("BTN_MNU"); break;
    //    case BTN_INC: Serial.print("BTN_INC"); break;
    //    case BTN_DEC: Serial.print("BTN_DEC"); break;
    //    case BTN_ESC: Serial.print("BTN_ESC"); break;
    //    default: Serial.print("???"); break;
    //}
    //Serial.print(", BS_STATE was ");
    //Serial.print(bsState);

    switch (buttonId) {
        case BTN_ESC:
            bsState = BS_WAITING;
            bsTaskState = TASK_UNKNOWN;
            sendFlightSettings(slope);
            break;

        case BTN_MNU:
            switch (bsState) {
                case BS_WAITING:        bsState = BS_SEL_GLIDER; break;
                case BS_AIRBORNE:       bsState = BS_SEL_GLIDER; break;
                case BS_ONGROUND:       bsState = BS_SEL_GLIDER; break;
                case BS_SEL_GLIDER:     bsState = BS_SEL_A_RIGHT; break;
                case BS_SEL_A_RIGHT:    bsState = BS_SEL_SLOPEANGLE; break;
                case BS_SEL_SLOPEANGLE: bsState = BS_SET_PLOC; break;
                case BS_SET_PLOC:       bsState = BS_SEL_GLIDER; break;
                default: break;
            }
            break;

        case BTN_INC:
            switch (bsState) {
                case BS_SEL_GLIDER:     slope.incrementGliderId(1); break;
                case BS_SEL_A_RIGHT:    slope.toggleABaseSide(); break;
                case BS_SEL_SLOPEANGLE: slope.incrementSlopeAngle(5); break;
                case BS_SET_PLOC:       sendPosRequest(slope); break;
                default: break;
            }
            break;

        case BTN_DEC:
            switch (bsState) {
                case BS_SEL_GLIDER:     slope.incrementGliderId(-1); break;
                case BS_SEL_A_RIGHT:    slope.toggleABaseSide(); break;
                case BS_SEL_SLOPEANGLE: slope.incrementSlopeAngle(-5); break;
                case BS_SET_PLOC:       sendPosRequest(slope); break;
                default: break;
            }
            break;
    }
    // Serial.print(" now ");
    // Serial.println(bsState);
}

void StateMachine::updateLCD() {
    char line1 [17];
    char line2 [17];
    bool rtkFix = true; // TODO: actual fix status
    int numSats = 16;   // TODO: replace with real GNSS data

    // build line 1
    snprintf(line1, sizeof(line1), "%s|%dsat|G%d|A%c",
        rtkFix ? "RTK" : "GPS",
        numSats,
        slope.getGliderId(),
        slope.getABaseLeft() ? 'L' : 'R');

    // build line 2
    switch (bsState) {
    case BS_WAITING:
        lcd.clear();
        strcpy(line2, "Ready to fly");
        break;
    case BS_ONGROUND:
        lcd.clear();
        strcpy(line2, "On ground");
        break;
    case BS_AIRBORNE:
        if (bsTaskState != TASK_FINISHED) {
            char line2[21];  // LCD-linje, 20 tegn + nullterminator

            const char* stateText = getTaskStateText(bsTaskState);
            String timeStr = formatTimeMSS((millis() - startTimestamp) / 1000);

            // Bygg strengen trygt
            snprintf(line2, sizeof(line2), "Airb %s %s", stateText, timeStr.c_str());
        }
        else {
            // Anta at formatTimeSSS() returnerer String – trekk ut c_str()
            String timeStr = formatTimeSSS(finishTimestamp - startTimestamp);

            snprintf(line2, sizeof(line2), "Fini %s", timeStr.c_str());
        }
        break;
    case BS_SEL_GLIDER:
        snprintf(line2, sizeof(line2), "Sel glider <%d>", slope.getGliderId());
        break;
    case BS_SEL_A_RIGHT:
        if (slope.getABaseLeft()) {
            strcpy(line2, "A base <left>");
        }
        else {
            strcpy(line2, "A base <right>");
        }
        break;
    case BS_SEL_SLOPEANGLE:
        snprintf(line2, sizeof(line2), "Slope <%d> deg", slope.getSlopeAngle());
        break;
    case BS_SET_PLOC: {
        int n, e, d, angle;
        float dist;
        slope.getPilotOffsetNED(n, e, d);
        directionAndDistance(n, e, d, angle, dist);
        snprintf(line2, sizeof(line2), "%dg %.1fm <ST>", angle, dist / 100.0f);
        break;
    }
    default:
        break;
    }

    padRight(line1, 16);
    padRight(line2, 16);

    lcd.setCursor(0, 0);
    lcd.print(line1);

    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void StateMachine::taskState(BSTaskState state) {
    bsTaskState = state;
}

BSTaskState StateMachine::taskState() const {
    return bsTaskState;
}

void StateMachine::baseState(BSState state) {
    bsState = state;
}

BSState StateMachine::baseState() const {
    return bsState;
}

const char* StateMachine::getTaskStateText(BSTaskState s) {
    switch (s) {
        case TASK_UNKNOWN:   return "Unkno";
        case TASK_OUTSIDE_A: return "Out A";
        case TASK_STARTED:   return "Start";
        case TASK_TURN1:     return "Leg02";
        case TASK_TURN2:     return "Leg03";
        case TASK_TURN3:     return "Leg04";
        case TASK_TURN4:     return "Leg05";
        case TASK_TURN5:     return "Leg06";
        case TASK_TURN6:     return "Leg07";
        case TASK_TURN7:     return "Leg08";
        case TASK_TURN8:     return "Leg09";
        case TASK_TURN9:     return "Leg10";
        case TASK_FINISHED:  return "Fini";
        default: return "";
    }
}

void StateMachine::setPilotOffset(int n, int e, int d) {
    slope.setPilotOffsetNED(n,e,d);
 
    #ifdef DEBUG
        Serial.print("Offset set ");
        Serial.print(n);
        Serial.print(" ");
        Serial.print(e);
        Serial.print(" ");
        Serial.println(d);

        slope.getPilotOffsetNED(n, e, d);

        Serial.print("Offset get ");
        Serial.print(n);
        Serial.print(" ");
        Serial.print(e);
        Serial.print(" ");
        Serial.println(d);
    #endif
}

void StateMachine::setGliderId(int gliderId) {
    slope.setGliderId(gliderId);
}

void StateMachine::setABaseLeft(bool isABaseLeft) {
    slope.setABaseLeft(isABaseLeft);
}
