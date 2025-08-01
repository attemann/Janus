#include "RTKF3F.h"

// UBX sync bytes and message details
#define UBX_SYNC1            0xB5
#define UBX_SYNC2            0x62
#define UBX_CLASS_NAV        0x01
#define UBX_ID_RELPOSNED     0x3C
#define UBX_RELPOSNED_LEN    40

#include "GNSSModule.h"
#include "RadioModule.h"

extern GNSSModule gnss;
extern RadioModule radioMod;






