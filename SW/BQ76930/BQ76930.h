//Set Address and CRC Enabled - See page 4 of Datasheet
#define BQ_ADD 0x08
#define CRC_EN 1
#define R_SNS_MOHM 0.75
#define MINTEMP 0.0
#define MAXTEMP 40.0
#define EE_LASTERR 0x0F

// vv Dont touch it - sonst klatscht it vv
#define BQ769X0_DEBUG
#ifdef BQ769X0_DEBUG
    #define LOG_PRINT(x)    //Serial.print(x)
    #define LOG_PRINTLN(x)  //Serial.println(x)
#else
    #define LOG_PRINT(x)
    #define LOG_PRINTLN(x)
#endif

#include <Arduino.h>
#include <Wire.h>     // I2C/TWI (for Battery Management IC)
#include <math.h>     // log for thermistor calculation
#include <SoftwareSerial.h> //Comm to outside world
#include <EEPROM.h>

#include "regmap.h"
#ifndef APP
  #define APP
  #include "app.h"
#endif

//Global variables
extern int32_t adcoffset;
extern uint16_t adcgain;
extern long rawCellVoltages[16];
extern float cellVoltages[7];
extern float packVoltage;
extern float packCurrent;
extern float accumCurrent;
extern float packTemp;
extern unsigned long lastCheck;
extern bool errFlag;

//Functions
#ifdef BQ769X0_DEBUG
  const char *byte2char(int x);
#endif

uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData);

void BQINT();

void writeReg(byte address, int data);
int readReg(byte address);

void initBQ();
void pinInit();
void initComm();

bool enableFets();
void disableFets();

long voltageFromAdc(uint16_t adc);
uint16_t adcFromVoltage(long voltage);

void measureCells();
void mapCells();
void measurePack();

void measureTemp();

int checkStatus();
void doCC();

void float2Bytes(byte bytes_temp[4],float float_variable);
void bytes2Float(float* float_variable,byte bytes_temp[4]);
