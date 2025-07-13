/*
  Enable an RTCM message on various ports, at various rates.
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 2nd, 2023
  License: MIT. Please see LICENSE.md for more information.

  This example shows how to put the UM980 into a Base mode configuration using specified coordinates.
  These examples are targeted for an ESP32 platform but any platform that has multiple
  serial UARTs should be compatible.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Triband GNSS RTK Breakout - UM980 (GPS-23286) https://www.sparkfun.com/products/23286

  Hardware Connections:
  Connect RX2 of the UM980 to pin 4 on the ESP32
  Connect TX2 of the UM980 to pin 13 on the ESP32
  To make this easier, a 4-pin locking JST cable can be purchased here: https://www.sparkfun.com/products/17240
  Note: Almost any ESP32 pins can be used for serial.
  Connect a dual or triband GNSS antenna: https://www.sparkfun.com/products/21801
*/

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS
#include <SparkFun_Extensible_Message_Parser.h> //http://librarymanager/All#SparkFun_Extensible_Message_Parser

//----------------------------------------
// Constants
//----------------------------------------

#define pin_UART_TX     17
#define pin_UART_RX     16

// Build the table listing all of the parsers
SEMP_PARSE_ROUTINE const parserTable[] =
{
    sempRtcmPreamble
};
const int parserCount = sizeof(parserTable) / sizeof(parserTable[0]);

const char * const parserNames[] =
{
    "RTCM parser"
};
const int parserNameCount = sizeof(parserNames) / sizeof(parserNames[0]);

//----------------------------------------
// Locals
//----------------------------------------

SEMP_PARSE_STATE *parse;
UM980 myGNSS;

HardwareSerial SerialGNSS(1); //Use UART1 on the ESP32

bool surveyStarted = false;
bool surveyComplete = false;
unsigned long lastCheck = 0;

int parseField(const String& line, int num) {
  //Serial.println("Parsing " + line);
  int start = line.indexOf(",") + 1;
  for (int i = 0; i < num; i++) {
    start = line.indexOf(",", start) + 1;
  }
  int end = line.indexOf(",", start);
  //Serial.print("start: "+String(start));
  //Serial.println(" end: "+String(end));

  return line.substring(start, end).toInt();
}
  
String gnggaFixTypeToString(int fixType) {
  switch (fixType) {
    case 0: return "Invalid (no fix)";
    case 1: return "GPS fix";
    case 2: return "DGPS fix";
    case 3: return "PPS fix";
    case 4: return "RTK Float";
    case 5: return "RTK Fixed";
    case 6: return "Dead Reckoning";
    case 7: return "Manual Input Mode";
    case 8: return "Simulation Mode";
    default: return "Unknown Fix Type";
  }
}

// ---------- Parse RTKSTATUSA responses ----------
bool surveyInComplete() {
    String line = SerialGNSS.readStringUntil('\n');
    line.trim();

    if (line.startsWith("$GNGGA")) {
      //Serial.println("RTKSTATUSA: " + line);

      int posType = parseField(line,5);
      int SIV = parseField(line,6);

      Serial.print("posType " + String(posType) + " [" + gnggaFixTypeToString(posType) + "]");
      Serial.println(", SIV " + String(SIV));

      /*
      if (posType == 1) { // 1 = Survey-In
        Serial.printf("Survey-In in progress: %ds, acc=%.2fm\n", duration, acc);
        return false;
      } else if ((rtkState == 0 || rtkState == 5) && acc > 0 && acc < 2.0) {
        // 0 = Single, 5 = Fixed RTK
        Serial.println("Survey-In complete!");
        return true;
      } else {
        Serial.println("Not in survey-in, not ((rtkState == 0 || rtkState == 5) && acc > 0 && acc < 2.0)");
        return false;
      }
      */
    }
    return false;
  }

// ---------- Configure RTCM output after Survey-In ----------
void configureRTCM() {
  myGNSS.setRTCMPortMessage("RTCM1006", "COM2", 1);
  myGNSS.setRTCMPortMessage("RTCM1033", "COM2", 10);
  myGNSS.setRTCMPortMessage("RTCM1074", "COM2", 1);
  myGNSS.setRTCMPortMessage("RTCM1084", "COM2", 1);
  myGNSS.setRTCMPortMessage("RTCM1094", "COM2", 1);
  myGNSS.setRTCMPortMessage("RTCM1124", "COM2", 1);
  myGNSS.sendCommand("saveconfig");
}

void setup()
{
  Serial.begin(115200);
  delay(250);
  Serial.println();
  Serial.println("SparkFun UM980 Example 5");

  // Initialize the parser
  parse = sempBeginParser(parserTable, parserCount,
                          parserNames, parserNameCount,
                          0, 3000, processMessage, "RTCM_Test");
  if (!parse)
    reportFatalError("Failed to initialize the parser");
  sempEnableDebugOutput(parse);

  //We must start the serial port before using it in the library
  SerialGNSS.begin(115200, SERIAL_8N1, pin_UART_RX, pin_UART_TX);

  myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(SerialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates.");
    while (1);
  } else Serial.println("UM980 detected, starting survey-in");

  //myGNSS.sendCommand("reset");
  //delay(2000);

  myGNSS.sendCommand("unlog");
  myGNSS.sendCommand("gpgga com1 1"); // For debug
  myGNSS.sendCommand("gpgga com2 1"); // For esp32

  



  // ----- Start Survey-In -----
  const uint32_t surveyDurationSec = 30;      // 1 min
  const float surveyAccuracyMeters = 1;     // 0.5 meter accuracy target
  String cmd = "mode base time " + String(surveyDurationSec) + " " + String(surveyAccuracyMeters);
  myGNSS.sendCommand(cmd.c_str());
  delay(1000);




  surveyStarted = true;

  // ----- Wait for Survey-In to complete -----
  Serial.print("Survey-In in progress");
  while (!surveyInComplete())
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println("Survey-In complete!");

  myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  Serial.println("RTCM messages are dumped in HEX if the CRC is correct");
}

void loop()
{
  while (SerialGNSS.available())
    // Update the parser state based on the incoming byte
    sempParseNextByte(parse, SerialGNSS.read());
}

// Call back from within parser, for end of message
// Process a complete message incoming from parser
void processMessage(SEMP_PARSE_STATE *parse, uint16_t type)
{
  SEMP_SCRATCH_PAD *scratchPad = (SEMP_SCRATCH_PAD *)parse->scratchPad;
  static bool displayOnce = true;

  // Display the raw message
  Serial.println();
  Serial.printf("Valid RTCM message: 0x%04x (%d) bytes\r\n", parse->length, parse->length);
  ex5DumpBuffer(parse->buffer, parse->length);

  // Display the parser state
  if (displayOnce)
  {
    displayOnce = false;
    Serial.println();
    sempPrintParserConfiguration(parse);
  }
}

// Display the contents of a buffer
void ex5DumpBuffer(const uint8_t *buffer, uint16_t length)
{
  int bytes;
  const uint8_t *end;
  int index;
  uint16_t offset;

  end = &buffer[length];
  offset = 0;
  while (buffer < end)
  {
    // Determine the number of bytes to display on the line
    bytes = end - buffer;
    if (bytes > (16 - (offset & 0xf)))
      bytes = 16 - (offset & 0xf);

    // Display the offset
    Serial.printf("0x%08lx: ", offset);

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      Serial.printf("   ");

    // Display the data bytes
    for (index = 0; index < bytes; index++)
      Serial.printf("%02x ", buffer[index]);

    // Separate the data bytes from the ASCII
    for (; index < (16 - (offset & 0xf)); index++)
      Serial.printf("   ");
    Serial.printf(" ");

    // Skip leading bytes
    for (index = 0; index < (offset & 0xf); index++)
      Serial.printf(" ");

    // Display the ASCII values
    for (index = 0; index < bytes; index++)
      Serial.printf("%c", ((buffer[index] < ' ') || (buffer[index] >= 0x7f)) ? '.' : buffer[index]);
    Serial.printf("\r\n");

    // Set the next line of data
    buffer += bytes;
    offset += bytes;
  }
}

// Print the error message every 15 seconds
void reportFatalError(const char *errorMsg)
{
  while (1)
  {
    Serial.print("HALTED: ");
    Serial.print(errorMsg);
    Serial.println();
    sleep(15);
  }
}
