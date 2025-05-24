//Receiver.cpp
#include <RTKF3F.h>
#include "Receiver.h"
#include "StateMachine.h"
#include "MessageTypes.h"  // includes decoding helpers

// Assume a global or external reference to your state machine instance
extern StateMachine taskStateMachine;

void handleIncomingMessage(const uint8_t* msg, size_t len) {
  if (len != 6) return;

  uint8_t msgType = decodeMsgType(msg[0]);
  uint8_t gliderId = decodeGliderId(msg[0]);

  switch (msgType) {
    case MSG_TYPE_G2B_EVENT: {
      uint8_t event, status;
      decodeEventMessage(msg, gliderId, event, status);
      taskStateMachine.handleEvent(static_cast<EventCode>(event));
      break;
    }

    case MSG_TYPE_G2B_RELPOS: {
      int16_t n, e, d;
      decodeRelPos(msg, n, e, d);

      taskStateMachine.setPilotOffset(n, e, d);  // store in slope
      Serial.printf("Offset set: N=%d, E=%d, D=%d\n", n, e, d);
      break;
    }

    case MSG_TYPE_G2B_MISC: {
      uint8_t d;
      decodeMiscMessage(msg, gliderId, d);
      Serial.printf("Received msg (code 0x%02X) from glider %u\n", d, gliderId);
      // TODO: handle d if needed
      break;
    }

    default:
      // Unknown message type
      Serial.println("Unknown message type received");
      break;
  }

  // TODO: trigger beeper or log alert if needed
}
