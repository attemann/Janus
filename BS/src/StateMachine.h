// StateMachine.h
#pragma once

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "RTKF3F.h"
#include "SendCmd.h"


class StateMachine {
public:
    void begin();
    void handleEvent(EventCode event);
    void handleButton(int buttonId);
    void updateLCD();

    void baseState(BSState state); 
    BSState baseState() const;     

    void taskState(BSTaskState state); 
    BSTaskState taskState() const;

    void setGliderId(int gliderId);
    void setPilotOffset(int n, int e, int d);
    void setABaseLeft(bool isABaseLeft);

private:
    void nextState();
    void previousState();
    const char* getTaskStateText(BSTaskState s);

    BSState bsState = BS_WAITING;
    BSTaskState bsTaskState = TASK_UNKNOWN;

    unsigned long startTimestamp = 0;
    unsigned long finishTimestamp = 0;
};

#endif
