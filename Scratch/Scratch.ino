//Scratch.ino
#include <Arduino.h>
#include <RFM69.h>
#include <RadioModule.h>
#include <SparkFun_Unicore_GNSS_Arduino_Library.h>

#define APPNAME "Scratch 1.0"

#define GNSS_BAUD 115200
#define UART_RX       4
#define UART_TX       5

UM980 myGNSS;
HardwareSerial SerialGNSS(1);

#define FREQUENCY_RTCM 868100000 
#define FREQUENCY_CD   868200000



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
    .reset = RFM69_RST,
    .irqn = RFM69_IRQN
};
unsigned long lastCheck = 0;

RFM69 radio(radioPins.cs, radioPins.irq, true);
RadioModule radioMod(radio);

void setup()
{
    Serial.begin(115200);
    delay(250);
    Serial.println();
    Serial.println("SparkFun UM980 Example 1");

    //We must start the serial port before using it in the library
    SerialGNSS.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

    //myGNSS.enableDebugging(); // Print all debug to Serial

    if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
    {
        Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
        while (true);
    }

    myGNSS.initBestnav();
    myGNSS.setModeRoverSurvey();
    myGNSS.saveConfiguration();
    if (!radioMod.init(radioPins, NODEID_GU, NETWORK_ID, FREQUENCY_CD)) {
        Serial.println(" Radio init failure, freeze");
        while (true);
    }
    else Serial.println("Radio init ok");

    if (!radioMod.verify()) {
        Serial.println(" Radio init failure, freeze");
        while (true);
    }
    else Serial.println("Radio verified");

    radioMod.sendMessageCode(NODEID_CD, FREQUENCY_CD, FREQUENCY_RTCM, MSG_INFORMATION, INFO_DEVICE_STARTING);

    //Turn off all NMEA, RTCM, and any other message that may be reporting periodically
    myGNSS.disableOutput();

    Serial.print("Model Type: ");
    Serial.print(myGNSS.getModelType());
    Serial.println();

    Serial.print("Unique Module ID: ");
    Serial.print(myGNSS.getID());
    Serial.println();

    Serial.print("Firmware Version: ");
    Serial.print(myGNSS.getVersion());
    Serial.println();

    Serial.print("Firmware Compile Time: ");
    Serial.print(myGNSS.getCompileTime());
    Serial.println();

    Serial.print("Full Version Report: ");
    Serial.print(myGNSS.getVersionFull());
    Serial.println();

    while (1);
}


void loop()
{
    myGNSS.update(); //Regularly call to parse any new data

    if (millis() - lastCheck > 1000)
    {
        lastCheck = millis();

        //The get methods are updated whenever new data is parsed with the update() call.
        //By default, this data is updated once per second.

        Serial.print("Lat/Long/Alt: ");
        Serial.print(myGNSS.getLatitude(), 11); //Accurate 11 decimal places
        Serial.print("/");
        Serial.print(myGNSS.getLongitude(), 11);
        Serial.print("/");
        Serial.print(myGNSS.getAltitude(), 4); //Accurate to 4 decimal places
        Serial.println();

        Serial.print("Horizontal Speed: ");
        Serial.print(myGNSS.getHorizontalSpeed());
        Serial.print("m/s Vertical Speed: ");
        Serial.print(myGNSS.getVerticalSpeed());
        Serial.print("m/s Direction from North: ");
        Serial.print(myGNSS.getTrackGround());
        Serial.print("(degrees)");
        Serial.println();

        Serial.print("Date (yyyy/mm/dd): ");
        Serial.print(myGNSS.getYear());
        Serial.print("/");
        if (myGNSS.getMonth() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getMonth());
        Serial.print("/");
        if (myGNSS.getDay() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getDay());

        Serial.print(" Time (hh:mm:dd): ");
        if (myGNSS.getHour() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getHour());
        Serial.print(":");
        if (myGNSS.getMinute() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getMinute());
        Serial.print(":");
        if (myGNSS.getSecond() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getSecond());
        Serial.print(".");
        if (myGNSS.getMillisecond() < 100)
            Serial.print("0");
        if (myGNSS.getMillisecond() < 10)
            Serial.print("0");
        Serial.print(myGNSS.getMillisecond());
        Serial.println();

        Serial.print("Satellites in view: ");
        Serial.print(myGNSS.getSIV());
        Serial.print(" PosType: ");

        Serial.println(myGNSS.getPositionType());
        Serial.println();

        Serial.println();
    }
}