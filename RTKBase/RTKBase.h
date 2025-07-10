#pragma once


enum BASESTATE {
    BASE_IDLE,
    BASE_SURVEYING,
    BASE_OPERATING,
    BASE_MENU
};

BASESTATE baseState = BASESTATE::BASE_IDLE;




bool parseGGAStatus(const String& line, int& fix, int& sats, float& HDOP) {

    Serial.println(line);

    if (!line.startsWith("$GPGGA") && !line.startsWith("$GNGGA")) return false;

    int fieldIndex = 0;
    int lastPos = 0;
    int nextPos = 0;
    String fields[15];  // GGA typically has up to 15 fields

    // Split line into fields
    while ((nextPos = line.indexOf(',', lastPos)) != -1 && fieldIndex < 15) {
        fields[fieldIndex++] = line.substring(lastPos, nextPos);
        lastPos = nextPos + 1;
    }
    fields[fieldIndex] = line.substring(lastPos);  // Add last field

    if (fieldIndex < 7) return false;  // Not enough fields

    fix = fields[6].toInt();
    sats = fields[7].toInt();
    HDOP = fields[8].toFloat();

    //Serial.printf("Parsed GGA: fix quality = %d, satellites = %d, HDOP=%f4.2\n", fix, sats, HDOP);
    return true;
}

void enqueueCommand(const String& cmd) {
    int nextTail = (queueTail + 1) % MAX_COMMANDS;
    if (nextTail != queueHead) { // Avoid overflow
        commandQueue[queueTail] = cmd;
        queueTail = nextTail;
    }
    else {
        Serial.println("[CMD QUEUE] Full!");
    }
}

void processCommandQueue() {
    if (commandPending) {
        // Timeout?
        if (millis() - commandSentTime > MAX_RESPONSE_TIME) {
            Serial.println("[CMD TIMEOUT]");
            commandPending = false;
        }
        return;
    }

    if (queueHead != queueTail) {
        String cmd = commandQueue[queueHead];
        queueHead = (queueHead + 1) % MAX_COMMANDS;

        GNSSSerial.println(cmd);
        lastCommandSent = cmd;
        commandPending = true;
        commandSentTime = millis();

        Serial.print("[CMD SEND] ");
        Serial.println(cmd);
    }
}
