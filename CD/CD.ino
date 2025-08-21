/*
 Name:		CD.ino
 Created:	8/4/2025 9:01:34 PM
 Author:	JanOlavEndrerud
*/

#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>  

#include <RTKF3F.h>
#include <RadioTask.h>
#include <SoundTask.h>

#define APPNAME      "CD 1.0"  
#define MY_WAV_FILE  "/cd.unit.wav"

#define THIS_NODE_ID NODEID_CD

extern RadioModule* radioMod;
inline constexpr int8_t RFM69_MISO= 19;
inline constexpr int8_t RFM69_MOSI= 23;
inline constexpr int8_t RFM69_SCK = 18;
inline constexpr int8_t RFM69_CSS =  5;
inline constexpr int8_t RFM69_IRQ =  4;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST = -1;

extern RadioModule* radioMod;

SoundTask sound;

void haltUnit(String msg1, String msg2) {
    Serial.print(msg1);
    Serial.print(":");
    Serial.println(msg2);
    while (true);
}

void setup() {

    Serial.begin(115200);
    while (!Serial);
    delay(500);

    Serial.printf("%s starting\r\n", APPNAME);
    sound.begin(
        /*gain*/ 0.8f,
        /*bclk*/ 26,
        /*lrclk*/22,
        /*din*/  25,
        /*prio*/ 2,
        /*stackWords*/ 4096,
        /*core*/ 1
    );

	sound.send({ SoundCmdType::WAV, 0, 0.0f, MY_WAV_FILE }, 0);
	sound.send({ SoundCmdType::WAV, 0, 0.0f, "/starting.wav" }, 0);

    // Radio
    if (!radioStartTask(RFM69_MISO, RFM69_MOSI, RFM69_SCK,RFM69_CSS, RFM69_IRQ,
                        THIS_NODE_ID, NETWORK_ID, FREQUENCY_CD)) {
        sound.send({ SoundCmdType::WAV, 0, 0.0f, "/radio.wav" }, 0);
        sound.send({ SoundCmdType::ERROR, ERR_RADIOINIT, 0.0f }, 0);
        while (true);  // halt
    }
    Serial.println("Radio init ok");
}


void loop() {
    RxPacket pkt;
    if (radioReceive(pkt, 0)) {          // non-blocking; use portMAX_DELAY to block
       
        switch (pkt.from) {
		    case NODEID_RTKBASE:
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/RTK_base.wav" }, 0);
            break;
		    case NODEID_GU:      
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/glider_unit.wav" }, 0);
            break;
		    default:
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/unknown.wav" }, 0);
            break;
        }

        Serial.printf("RX from %u, len=%u, RSSI=%d\r\n", pkt.from, pkt.len, pkt.rssi);

        if (pkt.len == 2) {
            switch (pkt.data[0]) {
            case MSG_DEVICESTATE:
                switch (pkt.data[1]) {
                    case DEVICE_STARTING:   sound.send({ SoundCmdType::STARTING },  0); break;
                    case DEVICE_GETTINGFIX: sound.send({ SoundCmdType::GET_FIX },   0); break;
                    case DEVICE_SURVEYING:  sound.send({ SoundCmdType::SURVEY },    0); break;
                    case DEVICE_OPERATING:  sound.send({ SoundCmdType::OPERATING }, 0); break;
                default: sound.send({ SoundCmdType::ERROR, ERR_UNKNOWN, 0.0f },     0); break;
            }
            break;
            case MSG_ERROR:
                switch (pkt.data[1]) {
                    case ERR_RADIOINIT: sound.send({ SoundCmdType::ERROR, ERR_RADIOINIT }, 0); break;
                    case ERR_UART:      sound.send({ SoundCmdType::ERROR, ERR_UART }, 0);      break;
                    default:            sound.send({ SoundCmdType::ERROR, pkt.data[1] }, 0);   break;
                }
            break;
            case MSG_SIV:
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/gps.wav" }, 0);
                sound.send({ SoundCmdType::INT, pkt.data[1] }, 0);
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/satellites.wav" }, 0);
            break;
            case MSG_FIXTYPE:
                sound.send({ SoundCmdType::WAV, 0, 0.0f, "/fixtype.wav" }, 0);
                sound.send({ SoundCmdType::FIX, pkt.data[1] }, 0);
            break;
            default:
                Serial.printf("Unknown msg [%02X] code [%02X] from %u\n",
                pkt.data[0], pkt.data[1], pkt.from);
                sound.send({ SoundCmdType::ERROR, ERR_UNKNOWN, 0.0f }, 0);
            break;
            }
        }
    }
}
