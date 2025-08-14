//NTRIP.ino
#include <Arduino.h>
#include <RFM69.h>
#include <RTKF3F.h>

// Radio
#define RFM69_CS 21
#define RFM69_IRQ 16
#define RFM69_IRQN digitalPinToInterrupt(RFM69_IRQ)
#define RFM69_RST -1
#define RFM69_SCK 19
#define RFM69_MISO 20
#define RFM69_MOSI 18

RadioModule::HWPins radioPins = {
    .sck = RFM69_SCK,  // SCK pin
    .miso = RFM69_MISO,  // MISO pin
    .mosi = RFM69_MOSI,  // MOSI pin
    .cs = RFM69_CS,   // CS pin
    .irq = RFM69_IRQ,   // IRQ pin
    .reset = -1,  // Reset pin (not used)
    .irqn = digitalPinToInterrupt(RFM69_IRQ)  // Interrupt number for IRQ pin
};

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);

#include <SparkFun_Unicore_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_Unicore_GNSS
UM980 myGNSS;

#define pin_UART_TX 5
#define pin_UART_RX 4

HardwareSerial SerialGNSS(1);  //Use UART1 on the ESP32

//The ESP32 core has a built in base64 library but not every platform does
//We'll use an external lib if necessary.
#if defined(ARDUINO_ARCH_ESP32)
#include "base64.h"  //Built-in ESP32 library
#else
#include <Base64.h>  //nfriendly library from https://github.com/adamvr/arduino-base64, will work with any platform
#endif

//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceivedRTCM_ms = 0;        //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000;  //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

bool transmitLocation = true;         //By default we will transmit the units location via GGA sentence.
int timeBetweenGGAUpdate_ms = 10000;  //GGA is required for Rev2 NTRIP casters. Don't transmit but once every 10 seconds
long lastTransmittedGGA_ms = 0;

//Used for GGA sentence parsing from incoming NMEA
bool ggaSentenceStarted = false;
bool ggaSentenceComplete = false;
bool ggaTransmitComplete = false;  //Goes true once we transmit GGA to the caster

char ggaSentence[128] = { 0 };
byte ggaSentenceSpot = 0;
int ggaSentenceEndSpot = 0;

//void processNMEA(char incoming);
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void haltUnit(String msg1, String msg2) {
    Serial.print(msg1);
    Serial.print(":");
    Serial.println(msg2);
    while (true);
}


void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.println();
    Serial.println("SparkFun UM980 Example");

    //We must start the serial port before using it in the library
    SerialGNSS.begin(115200, SERIAL_8N1, pin_UART_RX, pin_UART_TX);

    //myGNSS.enableDebugging(); // Print all debug to Serial

    if (myGNSS.begin(SerialGNSS) == false)  //Give the serial port over to the library
    {
        Serial.println("UM980 failed to respond. Check ports and baud rates.");
        while (1)
            ;
    }
    Serial.println("UM980 detected!");

    myGNSS.disableOutput();  // Disables all messages on this port

    myGNSS.setModeRoverSurvey();

    //Enable the basic 5 NMEA sentences including GGA for the NTRIP Caster at 1Hz
    myGNSS.setNMEAPortMessage("GPGGA", "com2", 1);
    //myGNSS.setNMEAPortMessage("GPGSA", "com2", 1);
    //myGNSS.setNMEAPortMessage("GPGST", "com2", 1);
    //myGNSS.setNMEAPortMessage("GPGSV", "com2", 1);
    //myGNSS.setNMEAPortMessage("GPRMC", "com2", 1);

    myGNSS.saveConfiguration();  //Save the current configuration into non-volatile memory (NVM)

    Serial.println("GNSS Configuration complete");

    //Clear any serial characters from the buffer
    while (Serial.available()) {
        Serial.read();
    }

    if (!radioMod.init(radioPins, NODEID_GU, NETWORK_ID, FREQUENCY_CD)) {
        haltUnit("Radio init", "Failure, freeze");
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        haltUnit("Radio verify", "Failure, freeze");
    }
    else Serial.println("Radio verified");

    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);

}

void loop() {
    ggaTransmitComplete = true;
    beginClient();
    delay(1000);
    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);
}


//Connect to the NTRIP Caster, receive RTCM, and push it to the GNSS module
void beginClient() {

    long rtcmCount = 0;

    //Write incoming NMEA back out to serial port and check for incoming GGA sentencebyt
    byte incoming;
    while (SerialGNSS.available()) {
        incoming = SerialGNSS.read();
        processNMEA(incoming);
    }

    delay(10);
}


//As each NMEA character comes in you can specify what to do with it
//We will look for and copy the GGA sentence
void processNMEA(char incoming) {
    //Take the incoming char from the GNSS and check to see if we should record it or not
    if (incoming == '$' && ggaTransmitComplete == true) {
        ggaSentenceStarted = true;
        ggaSentenceSpot = 0;
        ggaSentenceEndSpot = sizeof(ggaSentence);
        ggaSentenceComplete = false;
    }

    if (ggaSentenceStarted == true) {
        ggaSentence[ggaSentenceSpot++] = incoming;

        //Make sure we don't go out of bounds
        if (ggaSentenceSpot == sizeof(ggaSentence)) {
            //Start over
            ggaSentenceStarted = false;
        }
        //Verify this is the GGA setence
        else if (ggaSentenceSpot == 5 && incoming != 'G') {
            //Ignore this sentence, start over
            ggaSentenceStarted = false;
        }
        else if (incoming == '*') {
            //We're near the end. Keep listening for two more bytes to complete the CRC
            ggaSentenceEndSpot = ggaSentenceSpot + 2;
        }
        else if (ggaSentenceSpot == ggaSentenceEndSpot) {
            ggaSentence[ggaSentenceSpot] = '\0';  //Terminate this string
            ggaSentenceComplete = true;
            ggaTransmitComplete = false;  //We are ready for transmission

            Serial.print("GGA Parsed - ");
            Serial.println(ggaSentence);

            //Start over
            ggaSentenceStarted = false;
        }
    }
}
