#include <RTKF3F.h>

const char* getMessageName(uint8_t id) {
    switch (id) {
    case MSG_RTCM:             return "RTCM";
    case MSG_RTCM_FRAGMENT:    return "RTCMFRAGMENT";
    case MSG_ARENA_SETTINGS:   return "FLIGHT_SETTINGS";
    case MSG_GLIDER_SETTINGS:  return "GLIDER_SETTINGS";
    case MSG_REQ_POS:          return "REQ_POS";
    case MSG_GU_GPS_SETTINGS:  return "GU_GPS_SETTINGS";
    case MSG_INFORMATION:      return "INFORMATION";
    case MSG_ERROR:            return "ERROR";
    case MSG_SIV:              return "SIV";
    case MSG_TYPE_G2B_EVENT:   return "G2B_EVENT";
    case MSG_TYPE_G2B_RELPOS:  return "G2B_RELPOS";
    case MSG_TYPE_G2B_MISC:    return "G2B_MISC";
    default:                   return "UNKNOWN";
    }
}