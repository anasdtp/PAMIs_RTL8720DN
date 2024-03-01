#include "Arduino.h"

#if !defined(CONFIG_PLATFORM_8721D)
  #error Only for Ameba Realtek RTL8720DN, RTL8722DM and RTL8722CSM platform.
#endif

#define DEBUG_WIFI_WEBSERVER_PORT   Serial

// Debug Level from 0 to 4
#define _WIFI_LOGLEVEL_             3

#define BOARD_TYPE      "Rtlduino RTL8720DN"

#ifndef BOARD_NAME
  #if defined(ARDUINO_BOARD)
    #define BOARD_NAME    ARDUINO_BOARD
  #elif defined(BOARD_TYPE)
    #define BOARD_NAME    BOARD_TYPE
  #else
    #define BOARD_NAME    "Unknown Board"
  #endif  
#endif

#define SHIELD_TYPE       "RTL8720DN"

#define SIZE_FIFO (32*6)

typedef struct Message{
    int nb_msg, robot_id, order, arg1, arg2, arg3;
}Message;

extern Message rxMsg[SIZE_FIFO];
extern int FIFO_ecriture;

void printWifiStatus();
void WifiLoop();

void setup_Wifi();
void remplirMessageStruct(const String& response);