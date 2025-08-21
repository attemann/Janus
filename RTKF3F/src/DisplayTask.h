// DisplayTask.h
#pragma once

#include <Arduino.h>

struct DisplayMessage {
    char line1[17];
    char line2[17];
};

extern TaskHandle_t displayTaskHandle;
extern QueueHandle_t displayQueue;

void displayTask(void* pvParameters);
void startDisplayTask();       // helper to create the task
void sendToDisplay(const String& l1, const String& l2);  // helper
void sendToDisplayIfChanged(const String& l1, const String& l2, uint32_t minIntervalMs);
