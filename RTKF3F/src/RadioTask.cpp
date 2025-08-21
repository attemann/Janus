//RadioTask.cpp
#include <Arduino.h>
#include "RadioTask.h"
#include "RadioModule.h"


// Global pointer if needed elsewhere
RadioModule* radioMod = nullptr;

static RadioInitParams radioParams;

// Task handle and queue
static TaskHandle_t radioTaskHandle = nullptr;

static QueueHandle_t radioQueue = nullptr;
constexpr size_t RADIO_QUEUE_SIZE = 10;

static QueueHandle_t radioRxQueue = nullptr;
constexpr size_t RADIO_RX_QUEUE_SIZE = 10;

void radioTask(void* pvParameters) {
    RadioInitParams* params = reinterpret_cast<RadioInitParams*>(pvParameters);

    static RadioModule radioInstance(params->pinCS, params->pinIRQ, true);
    radioMod = &radioInstance;

    if (!radioMod->init(params->pinMISO, params->pinMOSI, params->pinSCK,
        params->nodeId, params->networkId, params->frequency)) {
        Serial.println("❌ Radio init failed");
        vTaskDelete(nullptr);
        return;
    }
    Serial.println("✅ Radio initialized");

    RadioMessage msg;
    uint8_t buf[64];

    for (;;) {
        // 1) Drain any pending TX work quickly (non-blocking)
        while (xQueueReceive(radioQueue, &msg, 0) == pdTRUE) {
            switch (msg.type) {
            case RadioMessageType::RTCM:
                radioMod->sendRTCM(msg.rtcm.data, msg.rtcm.length);
                break;
            case RadioMessageType::STATUS:
                radioMod->sendMessageCode(
                    msg.status.destNode,
                    msg.status.destFreq,
                    msg.status.returnFreq,
                    msg.status.msgType,
                    msg.status.code
                );
                break;
            }
        }

        // 2) Poll for RX from the radio
        size_t len = sizeof(buf);
        if (radioMod->receive(buf, len)) {  // note: len is OUT
            if (len > 0 && len <= sizeof(((RxPacket*)0)->data)) {
                RxPacket pkt;
                pkt.from = radioMod->getSenderId();
                pkt.len = static_cast<uint8_t>(len);
                pkt.rssi = 0x00;/*radioMod->getLastRSSI();*/   // if you have this
                memcpy(pkt.data, buf, len);

                // push to RX queue (drop if full)
                xQueueSend(radioRxQueue, &pkt, 0);
            }
        }

        // 3) Small yield to keep system smooth
        vTaskDelay(1);
    }
}

bool radioStartTask(int8_t cs, int8_t irq, int8_t miso, int8_t mosi, int8_t sck,
    uint8_t nodeId, uint8_t networkId, uint32_t frequency) {
    if (radioTaskHandle) return true;  // Already started

    radioParams = { cs, irq, miso, mosi, sck, nodeId, networkId, frequency };

    radioQueue = xQueueCreate(RADIO_QUEUE_SIZE, sizeof(RadioMessage));
    if (!radioQueue) {
        Serial.println("❌ Failed to create radio queue");
        return false;
    }

    radioRxQueue = xQueueCreate(RADIO_RX_QUEUE_SIZE, sizeof(RxPacket));
    if (!radioRxQueue) {
        Serial.println("❌ Failed to create radio RX queue");
        vQueueDelete(radioQueue);
        radioQueue = nullptr;
        return false;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        radioTask, "RadioTask", 4096, &radioParams, 1, &radioTaskHandle, 1);

    if (result != pdPASS) {
        Serial.println("❌ Failed to start radio task");
        vQueueDelete(radioQueue);
        vQueueDelete(radioRxQueue);
        radioQueue = nullptr;
        radioRxQueue = nullptr;
        return false;
    }

    Serial.println("✅ Radio task started");
    return true;
}

bool radioSendRTCM(const uint8_t* data, size_t len) {
    if (!radioQueue || !data || len == 0) return false;

    RadioMessage msg;
    msg.type = RadioMessageType::RTCM;
    msg.rtcm.data = data;
    msg.rtcm.length = len;
    return xQueueSend(radioQueue, &msg, 0) == pdTRUE;
}

bool radioSendMsg(uint8_t destNode, uint32_t destFreq, uint32_t returnFreq, uint8_t msgType, uint8_t code) {
    if (!radioQueue) return false;

    RadioMessage msg;
    msg.type = RadioMessageType::STATUS;
    msg.status = { destNode, destFreq, returnFreq, msgType, code };
    return xQueueSend(radioQueue, &msg, 0) == pdTRUE;
}

bool radioReceive(RxPacket& out, TickType_t timeoutTicks) {
    if (!radioRxQueue) return false;
    return xQueueReceive(radioRxQueue, &out, timeoutTicks) == pdTRUE;
}