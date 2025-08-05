/*
 Name:		CD.ino
 Created:	8/4/2025 9:01:34 PM
 Author:	JanOlavEndrerud
*/

#include <Arduino.h>
#include <RFM69.h>  
#include <RTKF3F.h>
#include <RadioModule.h>
#include <Sound.h>

#define APPNAME  "CD 1.0"  
#define MY_WAV_FILE     "/cd.unit.wav"

// Radio
#define RFM69_CS 21
#define RFM69_IRQ 1
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1

#define RFM69_SCK 19
#define RFM69_MISO 20
#define RFM69_MOSI 18

RadioModule::HWPins radioPins = {
    .sck = RFM69_SCK,
    .miso = RFM69_MISO,
    .mosi = RFM69_MOSI,
    .cs = RFM69_CS,
    .irq = RFM69_IRQ,
    .reset = RFM69_RST
};

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);
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
    speaker.begin(0.1, 26, 22, 25);

    speaker.playWavFile("/RTCM.wav");
    speaker.speakInt(1006);
    delay(500);
    speaker.playWavFile("/RTCM.wav");
    speaker.speakInt(1084);
    delay(500);
    while (1);

    speaker.playWavFile(MY_WAV_FILE);
    speaker.playWavFile("/starting.wav");   
    delay(500);


    // Radio
    if (!radioMod.init(radioPins, NODEID_CD, NETWORK_ID, GU_TX_FREQ)) {
        speaker.playWavFile(MY_WAV_FILE);
        speaker.playWavFile("/radio.wav");
		speaker.speakError(ERROR_RADIO_INIT);
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        speaker.playWavFile(MY_WAV_FILE);
        speaker.playWavFile("/radio.wav");
        speaker.speakError(ERROR_RADIO_VERIFY);
        haltUnit("Radio verify", "Failure, freeze");
    }
    else Serial.println("Radio verified");
}


void loop() {
    uint8_t* data = nullptr;
    uint8_t len = 0;
    int senderId = 0;
    const char* originWav;

    if (radioMod.receive(data, len)) {
        if (len > 0) {

            senderId = radioMod.getSenderId();
            if (senderId == NODEID_RTKBASE) originWav = "/base.wav";
            if (senderId == NODEID_GU)      originWav = "/gu.unit.wav";

            switch (data[0]) {
            case MSG_INFORMATION:
                speaker.playWavFile(originWav);
                speaker.speakInfo(data[1]);
				Serial.printf("MSG_INFORMATION [%02X] from %d\n", data[1],senderId);
                break;
            case MSG_ERROR:
                speaker.playWavFile(originWav);
                speaker.speakError(data[1]);
                Serial.printf("MSG_ERROR       [%02X] from %d\n", data[1], senderId);
                break;
			case MSG_SIV:  
                speaker.playWavFile(originWav);
                speaker.playNumberFile(data[1]);
                speaker.playWavFile("/satellites.wav");
                Serial.printf("MSG_SIV         [%02X] from %d\n", data[1], senderId);
            default:
                speaker.playWavFile(originWav);
                speaker.speakError(ERROR_UNKNOWN);
                Serial.printf("Unknown message [%02X] from %d\n", data[1], senderId);
                break;
            }
        }
    }
}