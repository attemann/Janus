#include <RTKF3F.h>

const char* getMessageName(MessageType type) {
    switch (type) {
    case MessageType::MSG_RTCM:             return "RTCM";
    case MessageType::MSG_RTCM_FRAGMENT:    return "RTCMFRAGMENT";
    case MessageType::MSG_ARENA_SETTINGS:   return "FLIGHT_SETTINGS";
    case MessageType::MSG_GLIDER_SETTINGS:  return "GLIDER_SETTINGS";
    case MessageType::MSG_REQ_POS:          return "REQ_POS";
    case MessageType::MSG_GU_GPS_SETTINGS:  return "GU_GPS_SETTINGS";
    case MessageType::MSG_INFORMATION:      return "INFORMATION";
    case MessageType::MSG_ERROR:            return "ERROR";
    case MessageType::MSG_SIV:              return "SIV";
    case MessageType::MSG_G2B_EVENT:   return "G2B_EVENT";
    case MessageType::MSG_G2B_RELPOS:  return "G2B_RELPOS";
    case MessageType::MSG_G2B_MISC:    return "G2B_MISC";
    default:                   return "UNKNOWN";
    }
}