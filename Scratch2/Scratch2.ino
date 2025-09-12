#include <Arduino.h>
#include <HardwareSerial.h>

#define UART_RX 18
#define UART_TX 17

HardwareSerial SerialGNSS(2);

void debugRawGNSS(int timeoutMs = 5000) {
    Serial.println("=== Raw GNSS Data ===");
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (SerialGNSS.available()) {
            uint8_t c = SerialGNSS.read();
            if (c >= 32 && c <= 126) {  // Printable ASCII
                Serial.print((char)c);
            }
            else {
                Serial.printf("[%02X]", c);  // Show hex for non-printable
            }
        }
        delay(1);
    }
    Serial.println("\n=== End Raw Data ===");
}

void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("UM980 Communication Test");

    SerialGNSS.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
    delay(2000);  // Let UM980 boot

    // Test 1: Send version command
    Serial.println("Sending 'versiona' command...");
    SerialGNSS.println("versiona\r\n");
    debugRawGNSS(3000);

    // Test 2: Send unlog command
    Serial.println("Sending 'unlog' command...");
    SerialGNSS.println("unlog\r\n");
    debugRawGNSS(3000);
}

void loop() {
    // Echo everything between Serial and SerialGNSS
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        Serial.printf("Sending: %s\n", cmd.c_str());
        SerialGNSS.println(cmd);
        debugRawGNSS(2000);
    }
}