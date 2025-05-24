//BS.cpp

#include "BSx.h"
#include "StateMachine.h"

#include <LiquidCrystal_I2C.h>

#define REG_VERSION 0x10
#define EXPECTED_RFM69_VERSION 0x24

extern LiquidCrystal_I2C lcd;

bool verifyRadio(RFM69& radio, const char* label) {
  uint8_t version = radio.readReg(REG_VERSION);
  if (version != EXPECTED_RFM69_VERSION) {
    Serial.printf("%s radio not detected! REG_VERSION = 0x%02X\n", label, version);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String(label) + " Init");
    lcd.setCursor(0, 1);
    lcd.print("FAILED");
    return false;
  }
  return true;
}

void debugTaskStates(int sim) {
  if (sim == 0) taskStateMachine.baseState(BS_ONGROUND);
  if (sim == 1) taskStateMachine.baseState(BS_AIRBORNE);
  if (sim == 3) taskStateMachine.handleEvent(EVT_CROSS_A_OUT);
  if (sim == 4) taskStateMachine.handleEvent(EVT_CROSS_A_IN);
  if (sim == 5) taskStateMachine.handleEvent(EVT_CROSS_B_OUT);
  if (sim == 6) taskStateMachine.handleEvent(EVT_CROSS_A_OUT);  // 2 legs done
  if (sim == 7) taskStateMachine.handleEvent(EVT_CROSS_B_OUT);
  if (sim == 8) taskStateMachine.handleEvent(EVT_CROSS_A_OUT);  // 4 legs done
  if (sim == 9) taskStateMachine.handleEvent(EVT_CROSS_B_OUT);
  if (sim == 10) taskStateMachine.handleEvent(EVT_CROSS_A_OUT); // 6 legs done
  if (sim == 11) taskStateMachine.handleEvent(EVT_CROSS_B_OUT);
  if (sim == 12) taskStateMachine.handleEvent(EVT_CROSS_A_OUT); // 8 legs done
  if (sim == 13) taskStateMachine.handleEvent(EVT_CROSS_B_OUT);
  if (sim == 14) taskStateMachine.handleEvent(EVT_CROSS_A_OUT); // 10 legs done

  if (sim > 15) {
    slope.setGliderId(slope.getGliderId() + 1);
    slope.toggleABaseSide();
  }
}

void debugButtons(int sim) {
  if (sim == 1) taskStateMachine.handleButton(BTN_MNU);
  if (sim == 2) taskStateMachine.handleButton(BTN_MNU);
  if (sim == 3) taskStateMachine.handleButton(BTN_MNU);
  if (sim == 4) taskStateMachine.handleButton(BTN_MNU);
  if (sim == 5) taskStateMachine.handleButton(BTN_INC);
  if (sim == 6) taskStateMachine.handleButton(BTN_INC);
  if (sim == 7) taskStateMachine.handleButton(BTN_DEC);
  if (sim == 8) taskStateMachine.handleButton(BTN_DEC);
  if (sim == 9) taskStateMachine.handleButton(BTN_INC);
  if (sim == 10) taskStateMachine.handleButton(BTN_INC);
  if (sim == 11) taskStateMachine.handleButton(BTN_DEC);
  if (sim == 12) taskStateMachine.handleButton(BTN_DEC);

  if (sim > 12) {
    taskStateMachine.handleButton(BTN_ESC);
    taskStateMachine.handleButton(BTN_ESC);
    taskStateMachine.handleButton(BTN_ESC);
  }
}

String generateStars(int count) {
  count = constrain(count, 1, 5);
  String stars = "";
  for (int i = 0; i < count; i++) {
    stars += '>';
  }
  while (stars.length() < 5) {
    stars += ' ';
  }
  return stars;
}
