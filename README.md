# RTKF3F – RTK-Based F3F Glider Training System

**Author:** Jan Olav Endrerud  
**Email:** joe2817@gmail.com  
**Version:** 15 May 2025

---

## Overview

RTKF3F is a high-precision, low-latency training system for F3F radio-controlled gliders using RTK GNSS technology. It detects base crossings automatically and provides real-time feedback without needing manned base stations.

---

## Features

- **RTK precision (≤10 cm)** for consistent flight detection
- **<20 ms latency** between event and audio feedback
- Fully **offline system** (no internet needed)
- Detailed **post-flight analysis** from logged events
- Compact airborne unit with GNSS and 868 MHz radio
- LCD and button interface on base station
- Designed for use on slopes with configurable A/B base orientation

---

## System Components

### Base Station (BS)

- UM980 RTK GNSS (base mode)
- ESP32-S3 microcontroller
- 2× RFM69HCW radios (TX: RTCM, RX: events)
- 2-line I2C LCD display
- Beeper for feedback
- 4-button user interface
- USB or battery powered (10+ hrs)

### Glider Unit (GU)

- UM980 RTK GNSS (rover mode)
- Seeed Studio XIAO ESP32-C6
- RFM69HCW radio (868 MHz)
- Beeper for status indication
- Max size: 40 × 20 mm (excluding antennas)
- Powered from RC battery

---

## Flight Operation

1. **Setup**
   - Place BS and perform GNSS survey-in
   - Select slope angle and A-base side
   - Set pilot location (PLOC)

2. **Flight**
   - Glider launches and crosses into task zone
   - GU detects base crossings and sends event to BS
   - BS triggers beeps and logs events

3. **Post-flight**
   - Use logs for analysis and performance improvement

---

## GU ↔ BS Communication

| Direction | Message           | Description                             |
|----------:|-------------------|-----------------------------------------|
| BS → GU   | `MSG_FLIGHTSETTINGS` | Pilot offset, base side, slope angle    |
| BS → GU   | `MSG_RTCM`        | RTCM3 corrections (1 Hz)                |
| GU → BS   | `EVT_*` Events    | A/B base and safety line crossings      |
| GU → BS   | `EVT_ACK`         | Acknowledgement of received settings    |

**Event Codes:**
- `EVT_CROSS_A_IN`, `EVT_CROSS_B_OUT`, `EVT_AIRBORNE`, `EVT_LANDED`, etc.

---

## Wiring Overview

### Glider Unit (ESP32-C6)

| Pin     | Function             |
|--------:|----------------------|
| GPIO17  | UART TX → GNSS RX    |
| GPIO18  | UART RX ← GNSS TX    |
| GPIO10–12 | SPI (SCK, MOSI, MISO) |
| GPIO9   | SPI CS               |
| GPIO8   | RFM69 DIO0 (IRQ)     |

### Base Station (ESP32-S3)

| Pin     | Function                   |
|--------:|----------------------------|
| GPIO4/5 | UART to GNSS               |
| GPIO6–9 | SPI & DIO0 for RFM RX/TX   |
| GPIO18/19 | I2C LCD                   |
| GPIO20–23 | Buttons                  |

---

## Configuration Summary

### UM980 (Base)
```text
CFG-RTK-MODE = BASE
CFG-SURVEYIN-ENABLE = TRUE
CFG-SURVEYIN-MIN-DURATION = 120
CFG-SURVEYIN-POS-ACCURACY = 2.0
CFG-MSG UART2 RTCM3 1005/1074/1084/1094/1124 = 1Hz
```

### UM980 (Rover)
```text
CFG-RTK-MODE = ROVER
CFG-PORT UART2 = RTCM_INPUT
CFG-RATE = 25
CFG-MSG UART1 NAV-RELPOSNED = 25Hz
CFG-DYNMODEL = AIRBORNE 2g
```

---

## Development Roadmap

- [ ] Support for Jeti Ex-bus / F3FTool
- [ ] Safety line definition and events
- [ ] Optional voice/audio module

---

## License

MIT License

Copyright (c) 2025 Jan Olav Endrerud

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the “Software”), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
---

## Contact

Feel free to reach out with ideas or improvements:  
📧 joe2817@gmail.com