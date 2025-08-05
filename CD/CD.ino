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
#define I_AM     "/cd.unit.wav"

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

    speaker.playWavFile(I_AM);
    speaker.playWavFile("/starting.wav");   

    // Radio
    if (!radioMod.init(radioPins, NODEID_CD, NETWORK_ID, GU_TX_FREQ)) {
        speaker.playWavFile(I_AM);
		speaker.speakError(ERROR_RADIO_INIT);
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        speaker.playWavFile(I_AM);
        speaker.speakError(ERROR_RADIO_VERIFY);
        haltUnit("Radio verify", "Failure, freeze");
    }
    else Serial.println("Radio verified");
}


void loop() {
    speaker.speakTime(69.70);

    delay(500);
    speaker.playWavFile("/gu.unit.wav");
    speaker.playWavFile("/outside.wav");
    speaker.playWavFile("/Base_A.wav");
    delay(500);

    speaker.speakTime(13.42);
    delay(500);


}