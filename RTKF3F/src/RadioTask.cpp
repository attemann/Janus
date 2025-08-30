//RadioTask.cpp
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>

#include "RadioTask.h"
#include "RadioModule.h"

#ifndef ARDUINO_ARCH_ESP32
#error ESP32 only
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if (configNUM_CORES > 1)
#define CORE_APP   1
#define CORE_COMM  0
#define CORE_ANY   tskNO_AFFINITY
#else
#define CORE_APP   0
#define CORE_COMM  0
#define CORE_ANY   tskNO_AFFINITY
#endif


struct RadioInitParams {
    int8_t pinMISO;
    int8_t pinMOSI;
    int8_t pinSCK;
    int8_t pinCS;
    int8_t pinIRQ;
    uint8_t nodeId;
    uint8_t networkId;
    uint32_t frequency;
};

// === Globals ===
RadioModule* radioMod = nullptr;

static TaskHandle_t  radioTaskHandle = nullptr;

// TX control queue (STATUS)
static QueueHandle_t g_statusQ = nullptr;

// Bulk RTCM stream (GNSS → Radio)
static StreamBufferHandle_t g_rtcmStream = nullptr;

// Inbound radio frames (Radio → App)
static QueueHandle_t g_rxQ = nullptr;

// Preserve your init params
static RadioInitParams radioParams;

// === Task forward ===
static void radioTask(void* pvParameters);

// === Public API ===
bool radioStartTask(int8_t cs, int8_t irq, int8_t miso, int8_t mosi, int8_t sck,
    uint8_t nodeId, uint8_t networkId, uint32_t frequency)
{
    if (radioTaskHandle) return true; // already started

    radioParams = { cs, irq, miso, mosi, sck, nodeId, networkId, frequency };

    g_statusQ = xQueueCreate(RADIO_STATUS_Q_SIZE, sizeof(RadioMessageStatus));
    if (!g_statusQ) { Serial.println("❌ Failed to create STATUS queue"); return false; }

    g_rtcmStream = xStreamBufferCreate(RTCM_STREAM_BYTES, 64 /*trigger*/);
    if (!g_rtcmStream) {
        Serial.println("❌ Failed to create RTCM stream buffer");
        vQueueDelete(g_statusQ); g_statusQ = nullptr;
        return false;
    }

    g_rxQ = xQueueCreate(RADIO_RX_Q_SIZE, sizeof(RxPacket));
    if (!g_rxQ) {
        Serial.println("❌ Failed to create RX queue");
        vStreamBufferDelete(g_rtcmStream); g_rtcmStream = nullptr;
        vQueueDelete(g_statusQ); g_statusQ = nullptr;
        return false;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        radioTask, "RadioTask", 4096, &radioParams, 2, &radioTaskHandle, CORE_COMM
    );

    if (ok != pdPASS) {
        Serial.println("❌ Failed to start RadioTask");
        vQueueDelete(g_rxQ);         g_rxQ = nullptr;
        vStreamBufferDelete(g_rtcmStream); g_rtcmStream = nullptr;
        vQueueDelete(g_statusQ);     g_statusQ = nullptr;
        return false;
    }

    Serial.println("✅ Radio task started");
    return true;
}

bool radioTxRtcmWrite(const uint8_t* data, size_t len, TickType_t toTicks) {
    if (!g_rtcmStream || !data || !len) return false;
    size_t w = xStreamBufferSend(g_rtcmStream, data, len, toTicks);
    return w == len;
}

bool radioReceive(RxPacket& out, TickType_t toTicks) {
    if (!g_rxQ) return false;
    return xQueueReceive(g_rxQ, &out, toTicks) == pdTRUE;
}

bool radioSendMsg(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq,
    uint8_t msgType, uint8_t code)
{
    if (!g_statusQ) return false;
    RadioMessageStatus s{ destNode, destFreq, returnFreq, msgType, code };
    return xQueueSend(g_statusQ, &s, 0) == pdTRUE;
}

// Back‑compat: map old API to stream write
bool radioSendRTCM(const uint8_t* data, size_t len) {
    return radioTxRtcmWrite(data, len, 0);
}

// === Task ===
static void radioTask(void* pvParameters) {
    // Setup radio
    auto* params = reinterpret_cast<RadioInitParams*>(pvParameters);
    static RadioModule radioInstance(params->pinCS, params->pinIRQ, true);
    radioMod = &radioInstance;

    if (!radioMod->init(params->pinMISO, params->pinMOSI, params->pinSCK,
        params->nodeId, params->networkId, params->frequency)) {
        Serial.println("❌ Radio init failed");
        radioMod = nullptr;
        vTaskDelete(nullptr);
        return;
    }
    Serial.println("✅ Radio initialized");

    uint8_t rxBuf[RADIO_MAX_PAYLOAD];
    uint8_t rtcmChunk[RTCM_TX_CHUNK];

    for (;;) {
        // 1) RX: poll radio and forward to app RX queue
        size_t rxLen = sizeof(rxBuf);
        if (radioMod->receive(rxBuf, rxLen)) {            // len is IN/OUT (ref)
            if (rxLen && rxLen <= sizeof(rxBuf)) {
                RxPacket pkt{};
                pkt.from = radioMod->getSenderId();
                pkt.rssi = radioMod->getLastRSSI();       // if exposed; else 0
                pkt.len = (uint8_t)min(rxLen, (size_t)RADIO_MAX_PAYLOAD);
                memcpy(pkt.data, rxBuf, pkt.len);
                (void)xQueueSend(g_rxQ, &pkt, 0);         // drop if full
            }
        }

        // 2) Drain RTCM stream in chunks and transmit
        size_t avail = xStreamBufferBytesAvailable(g_rtcmStream);
        while (avail) {
            size_t take = min(avail, (size_t)sizeof(rtcmChunk));
            size_t rd = xStreamBufferReceive(g_rtcmStream, rtcmChunk, take, 0);
            if (rd) {
                // Your driver may fragment internally; if not, you may need to
                // split rd into <= RADIO_MAX_PAYLOAD chunks:
                size_t off = 0;
                while (off < rd) {
                    size_t n = min((size_t)RADIO_MAX_PAYLOAD, rd - off);
                    radioMod->sendRTCM(rtcmChunk + off, n);
                    off += n;
                }
            }
            avail = xStreamBufferBytesAvailable(g_rtcmStream);
        }

        // 3) Send STATUS/control frames from queue
        RadioMessageStatus s;
        while (xQueueReceive(g_statusQ, &s, 0) == pdTRUE) {
            radioMod->sendMessageCode(
                s.destNode, s.destFreq, s.returnFreq, s.msgType, s.code
            );
        }

        // 4) Small yield
        vTaskDelay(1);
    }
}