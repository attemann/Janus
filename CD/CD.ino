/*
 Name:		CD.ino
 Created:	8/4/2025 9:01:34 PM
 Author:	JanOlavEndrerud
*/

#include <Arduino.h>
#include <SPI.h>
#include <RFM69.h>  
#include <RTKF3F.h>
#include "RadioModule.h"
#include <Sound.h>

#define APPNAME  "CD 1.0"  
#define MY_WAV_FILE     "/cd.unit.wav"

#define THIS_NODE_ID NODEID_CD

inline constexpr int8_t RFM69_MISO= 19;
inline constexpr int8_t RFM69_MOSI= 23;
inline constexpr int8_t RFM69_SCK = 18;
inline constexpr int8_t RFM69_CSS =  5;
inline constexpr int8_t RFM69_IRQ =  4;
inline constexpr int8_t RFM69_IRQN = digitalPinToInterrupt(RFM69_IRQ);
inline constexpr int8_t RFM69_RST = -1;

RadioModule radioMod(RFM69_CSS, RFM69_IRQ, true);
DecimalSpeaker speaker;

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
    speaker.begin(0.8, 26, 22, 25);

    speaker.playWavFile(MY_WAV_FILE);
    speaker.playWavFile("/starting.wav");   
    delay(500);

    // Radio
    if (!radioMod.init(RFM69_MISO, RFM69_MOSI, RFM69_SCK,
        THIS_NODE_ID, NETWORK_ID, FREQUENCY_CD)) {
        Serial.println("Radio init failed");        
        speaker.playWavFile(MY_WAV_FILE);
        speaker.playWavFile("/radio.wav");
		speaker.speakError(ERROR_RADIO_INIT);
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");
}


void loop() {
    uint8_t data[64];
    size_t len = sizeof(data);
    int senderId = 0;
    const char* originWav;

    if (radioMod.receive(data, len)) {
        if (len > 0) {

            senderId = radioMod.getSenderId();
            Serial.printf("Got senderid=%d\r\n", senderId);
            if (senderId == NODEID_RTKBASE) originWav = "/RTK_base.wav";
            if (senderId == NODEID_GU)      originWav = "/glider_unit.wav";

            speaker.playWavFile(originWav);

            switch (data[0]) {
            case MSG_INFORMATION: 
                Serial.printf("MSG_INFORMATION [%02X] from %d\n", data[1], senderId);
                switch (data[1]) {
                case INFO_DEVICE_STARTING:
                    speaker.speakStarting();
                    break;
                case INFO_TRANSITION_GETTINGFIX:
                    speaker.speakGettingFix();
					speaker.speakFix(data[1] - 4);  
					break;
                case INFO_TRANSITION_SURVEYING:
                    speaker.speakSurveyIn();
                    break;
                case INFO_TRANSITION_OPERATING:
                    speaker.speakOperating();
                    break;
                case INFO_FIX_NOFIX:
				case INFO_FIX_GPS:
				case INFO_FIX_DGPS:
				case INFO_FIX_RTK_FLOAT:
				case INFO_FIX_RTK_FIX:
				case INFO_FIX_DEAD_RECKONING:
				case INFO_FIX_MANUAL:   
				case INFO_FIX_SIM:  
				case INFO_FIX_OTHER:    
				case INFO_FIX_PPS:
                    speaker.speakFix(data[1] - 4);
                    break;
                default:
                    Serial.printf("Unknown info code [%02X] from %d\n", data[1], senderId);
                    speaker.speakInfo(data[1]);
                    break;
                }
                break;
            case MSG_ERROR:
                Serial.printf("MSG_ERROR       [%02X] from %d\n", data[1], senderId);
                switch (data[1]) {
                    case ERROR_RADIO_INIT:
                        speaker.playWavFile("/radio.wav");
					    break;
                    case ERROR_UART:    
						speaker.playWavFile("/gps.wav");
                        break;
                    default:
                        break;
                }
                speaker.speakError(data[1]);

                break;
			case MSG_SIV: 
                Serial.printf("MSG_SIV         [%02X] from %d\n", data[1], senderId);

                speaker.playNumberFile(data[1]);
                speaker.playWavFile("/satellites.wav");
                break;
            default:
                Serial.printf("Unknown message [%02X] from %d\n", data[1], senderId);
                speaker.speakError(ERROR_UNKNOWN);
                break;
            }
        }
    }
}