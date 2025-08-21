// RadioTask.h
#pragma once

#include <Arduino.h>

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>

// === Existing includes ===
#include "RadioModule.h"

// ---- CONFIG ----
constexpr size_t RADIO_MAX_PAYLOAD = 64;   // RFM69 usable payload
constexpr size_t RADIO_STATUS_Q_SIZE = 16;
constexpr size_t RADIO_RX_Q_SIZE = 16;
constexpr size_t RTCM_STREAM_BYTES = 4096; // ring buffer for RTCM bytes
constexpr size_t RTCM_TX_CHUNK = 240;  // per-packet chunk to send over radio

// ---- Existing message types (keep) ----
enum class RadioMessageType : uint8_t { RTCM, STATUS };

struct RadioMessageStatus {
    uint8_t  destNode;
    uint32_t destFreq;
    uint32_t returnFreq;
    uint8_t  msgType;
    uint8_t  code;
};

struct RadioMessage {
    RadioMessageType type;
    union {
        struct { const uint8_t* data; size_t length; } rtcm; // deprecated path
        RadioMessageStatus status;
    };
};

// ---- RX packet to the app ----
struct RxPacket {
    uint8_t  from = 0;
    int16_t  rssi = 0;        // optional, 0 if not available
    uint8_t  len = 0;
    uint8_t  data[RADIO_MAX_PAYLOAD];
};

// ---- Public API ----
bool radioStartTask(int8_t cs, int8_t irq, int8_t miso, int8_t mosi, int8_t sck,
    uint8_t nodeId, uint8_t networkId, uint32_t frequency);

// Preferred (new) APIs
bool radioTxRtcmWrite(const uint8_t* data, size_t len, TickType_t toTicks = 0); // GNSS → Radio (bulk)
bool radioReceive(RxPacket& out, TickType_t toTicks = 0);                        // Radio → App

// Keep your existing helper (STATUS)
bool radioSendMsg(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
    uint8_t msgType, uint8_t code);

// Back‑compat shim (optional): map old radioSendRTCM(...) to stream write
bool radioSendRTCM(const uint8_t* data, size_t len);