#pragma once

#ifndef BS_H
#define BS_H

	#include <Arduino.h>
	#include <RFM69.h>
	#include <RTKF3F.h>

	// ESP32-WROOM-32 Pinout

	// Radio RFM69HCW SDI
	#define RFM69_CS       5
	#define RFM69_IRQ      4
	#define RFM9_SCK      18
	#define RFM9_MISO     19
	#define RFM9_MOSI     23  

	// GPS UART
	#define UART2_TX      17
	#define UART2_RX      16

	//LCD I2C
	#define LCD_SDA       21
	#define LCD_SCL       22

	// Buttons, LED, Buzzer
	#define BUZZER_PIN    14
	#define BTN_MNU       32
	#define BTN_INC       25
	#define BTN_DEC       33
	#define BTN_ESC       26

	// Ledige pinner
	// GPIO0  (can be used, but keep HIGH at boot if not flashing)
	// GPIO2  (can be used, but has boot strapping role — keep LOW at boot)
	// GPIO1  (avoid using unless you know what you're doing — affects flash voltage selection
	// GPIO13
	// GPIO15 (has boot function — must be LOW at boot)
	// GPIO27
	// GPIO34 (input only)
	// GPIO35 (input only)
	// GPIO36(VP) (input only)
	// GPIO39(VN) (input only)

	class StateMachine;
	class Slope;

	extern Slope slope;
	extern StateMachine taskStateMachine;

	void debugTaskStates(int sim);
	void debugButtons(int sim);
	String generateStars(int count);
	bool verifyRadio(RFM69& radio, const char* label);

#endif
