#include <RFM69.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

#define NODE_ID        1
#define NETWORK_ID     100
#define GATEWAY_ID     2
#define FREQUENCY      RF69_868MHZ

#define RFM69_CS    5
#define RFM69_IRQ   4
#define RFM69_SCK   18
#define RFM69_MISO  19
#define RFM69_MOSI  23
#define RFM69_RST   -1

RFM69 radio(RFM69_CS, RFM69_IRQ, true);

// LCD address 0x27, 16 chars, 2 lines
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define LED_BUILTIN 2

void setup() {
  Serial.begin(115200);
  delay(10);

    // Initialize LCD
  lcd.init();           // Initialize the LCD
  lcd.backlight();      // Turn on the backlight

  pinMode(LED_BUILTIN,OUTPUT);

  lcd.setCursor(0, 0);  // First column, first row
  lcd.print("Begin radio");

  SPI.begin(RFM69_SCK,RFM69_MISO,RFM69_MOSI,RFM69_CS); // SCK, MISO, MOSI, SS

  if (RFM69_RST != -1) {
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
  }

  Serial.println("Starting RFM69 transmitter...");

  lcd.setCursor(0, 0);  // First column, first row
  lcd.print("Init radio ");

  if (!radio.initialize(FREQUENCY, NODE_ID, NETWORK_ID)) {
    Serial.println("Radio initialization failed!");
    while (1);
  }

  radio.setHighPower();
  radio.encrypt(NULL);

  Serial.println("RFM69 initialized");
}

void loop() {
  static int counter = 1; // Static counter to persist across loops
  char msg[200]; // Buffer for message
  bool success = false;
  snprintf(msg, sizeof(msg), "Hei, her er en lang tekst. Sender melding nummer %d", counter); // Format message as "Hei 1", "Hei 2", etc.

  Serial.print("Sending: ");
  Serial.println(msg);

  lcd.setCursor(0, 0);  // First column, first row
  lcd.print("Send");

  success=radio.sendWithRetry(GATEWAY_ID, msg, strlen(msg)); // Send the message
  //success = radio.send(GATEWAY_ID, msg, strlen(msg)); // Send the message

  if (success) {
    Serial.println("ACK!!");
    lcd.setCursor(0, 1);  // First column, first row
    lcd.print("ACK!!");

  } else {
    Serial.println("NoACK");
    lcd.setCursor(0, 1);  // First column, first row
    lcd.print("NoACK");
  }

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2500);

  counter++; // Increment counter;
}
