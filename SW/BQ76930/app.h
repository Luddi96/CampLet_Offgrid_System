//#include <Arduino.h>
//#include <Wire.h>

#ifndef BQ
  #define BQ
  #include "BQ76930.h"
#endif

//LEDs
#define P_LED_OK 8
#define P_LED_VOL 7
#define P_LED_CUR 6
#define P_LED_ERR 5
//Alert Pin
#define P_ALERT 2
//Comms
#define P_SDA A4
#define P_SCL A5
#define P_MAN_TX 11
#define P_MAN_RX 12

void blinkStartup();
void blinkOk();
void blinkCommErr();
void blinkStat(int stat);

extern SoftwareSerial swSer;

void getSerial();
