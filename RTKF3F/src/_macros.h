#pragma once

#define WIFI

#define RADIO_DEBUG
#define GNSS_DEBUG
//#define WIFI_DEBUG

#ifdef RADIO_DEBUG
#define RDBG_BEGIN(baud) Serial.begin(baud)
#define RDBG_PRINT(x)    Serial.print(x)
#define RDBG_PRINTLN(x)  Serial.println(x)
#define RDBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define RDBG_BEGIN(baud)    ((void)0)
#define RDBG_PRINT(x)       ((void)0)
#define RDBG_PRINTLN(x)     ((void)0)
#define RDBG_PRINTF(...)    ((void)0)
#endif


#ifdef GNSS_DEBUG
	#define GDBG_BEGIN(baud) Serial.begin(baud)
	#define GDBG_PRINT(x)    Serial.print(x)
	#define GDBG_PRINTLN(x)  Serial.println(x)
	#define GDBG_PRINTF(...) Serial.printf(__VA_ARGS__)
	#define GDBG_WRITE(x)    Serial.write(x)

#else
	#define GDBG_BEGIN(baud)    ((void)0)
	#define GDBG_PRINT(x)       ((void)0)
	#define GDBG_PRINTLN(x)     ((void)0)
	#define GDBG_PRINTF(...)    ((void)0)
    #define GDBG_WRITE(x)       ((void)0)
#endif


#ifdef WIFI_DEBUG
	#define WDBG_BEGIN(baud) Serial.begin(baud)
	#define WDBG_PRINT(x)    Serial.print(x)
	#define WDBG_PRINTLN(x)  Serial.println(x)
	#define WDBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
	#define WDBG_BEGIN(baud)    ((void)0)
	#define WDBG_PRINT(x)       ((void)0)
	#define WDBG_PRINTLN(x)     ((void)0)
	#define WDBG_PRINTF(...)    ((void)0)
#endif

// Define ANSI color codes (disable if not supported)
#define ANSI_GREEN  "\x1b[32m"
#define ANSI_RED    "\x1b[31m"
#define ANSI_RESET  "\x1b[0m"