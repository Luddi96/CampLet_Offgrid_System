#include "app.h"

SoftwareSerial swSer(P_MAN_RX, P_MAN_TX);
bool lastCommShutdown = 0;

void blinkStartup()
{
  digitalWrite(P_LED_OK, HIGH);
  delay(200);
  //digitalWrite(P_LED_OK, LOW);
  digitalWrite(P_LED_VOL, HIGH);
  delay(200);
  //digitalWrite(P_LED_VOL, LOW);
  digitalWrite(P_LED_CUR, HIGH);
  delay(200);
  //digitalWrite(P_LED_CUR, LOW);
  digitalWrite(P_LED_ERR, HIGH);
  delay(200);
  digitalWrite(P_LED_ERR, LOW);
  digitalWrite(P_LED_OK, LOW);
  digitalWrite(P_LED_VOL, LOW);
  digitalWrite(P_LED_CUR, LOW);
}

void blinkOk()
{
  delay(200);
  digitalWrite(P_LED_OK, HIGH);
  delay(200);
  digitalWrite(P_LED_OK, LOW);
  delay(200);
  digitalWrite(P_LED_OK, HIGH);
  delay(200);
  digitalWrite(P_LED_OK, LOW);
}

void blinkCommErr()
{
  for(int i = 0; i < 10; i++)
  {
    digitalWrite(P_LED_VOL, HIGH);
    digitalWrite(P_LED_CUR, HIGH);
    digitalWrite(P_LED_ERR, HIGH);
    delay(50);
    digitalWrite(P_LED_VOL, LOW);
    digitalWrite(P_LED_CUR, LOW);
    digitalWrite(P_LED_ERR, LOW);
    delay(150);
  }
}

void blinkStat(int stat)
{
  if(stat == OVRD)
  {
    for(int i = 0; i < 3; i++)
    {
      digitalWrite(P_LED_ERR, HIGH);
      delay(250);
      digitalWrite(P_LED_ERR, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00010000);
  }

  if(stat == XREADY)
  {
    for(int i = 0; i < 10; i++)
    {
      digitalWrite(P_LED_ERR, HIGH);
      delay(250);
      digitalWrite(P_LED_ERR, LOW);
      delay(250);
      digitalWrite(P_LED_ERR, HIGH);
      delay(250);
      digitalWrite(P_LED_ERR, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00100000);
  }

  if(stat == S_UV)
  {
    for(int i = 0; i < 10; i++)
    {
      digitalWrite(P_LED_VOL, HIGH);
      delay(250);
      digitalWrite(P_LED_VOL, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00001000);
  }
  if(stat == S_OV)
  {
    for(int i = 0; i < 10; i++)
    {
      digitalWrite(P_LED_VOL, HIGH);
      delay(250);
      digitalWrite(P_LED_VOL, LOW);
      delay(250);
      digitalWrite(P_LED_VOL, HIGH);
      delay(250);
      digitalWrite(P_LED_VOL, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00000100);
  }

  if(stat == S_SCD)
  {
    for(int i = 0; i < 10; i++)
    {
      digitalWrite(P_LED_CUR, HIGH);
      delay(250);
      digitalWrite(P_LED_CUR, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00000010);
  }
  if(stat == S_OCD)
  {
    for(int i = 0; i < 10; i++)
    {
      digitalWrite(P_LED_CUR, HIGH);
      delay(250);
      digitalWrite(P_LED_CUR, LOW);
      delay(250);
      digitalWrite(P_LED_CUR, HIGH);
      delay(250);
      digitalWrite(P_LED_CUR, LOW);
      delay(500);
    }
    writeReg(SYS_STAT, 0b00000001);
  }
}

void pinInit()
{
  pinMode(P_LED_OK, OUTPUT);
  pinMode(P_LED_VOL, OUTPUT);
  pinMode(P_LED_CUR, OUTPUT);
  pinMode(P_LED_ERR, OUTPUT);

  pinMode(P_ALERT, INPUT);
  digitalWrite(P_ALERT, LOW);

  blinkStartup();
  
}

void initComm()
{
  Serial.begin(9600);
  swSer.begin(9600);
  Wire.begin();
}

void getSerial()
{
  if(swSer.available())
  {
    byte rec = swSer.read();
    byte tosend[4];
    switch(rec)
    {
      //cellVoltages
      case 0x02:  byte tosend_cell[28];
                  for(int i = 0; i < 7; i++)
                  {
                    float2Bytes(&tosend_cell[4*i], cellVoltages[i]);
                  }
                  swSer.write(tosend_cell, 28);
                  lastCommShutdown = 0;
                  break;
      //PackVoltage
      case 0x03:  float2Bytes(tosend, packVoltage);
                  swSer.write(tosend, 4);
                  lastCommShutdown = 0;
                  break;
      //PackCurrent
      case 0x04:  float2Bytes(tosend, packCurrent);
                  swSer.write(tosend, 4);
                  lastCommShutdown = 0;
                  break;
      //AccumCurrent
      case 0x05:  float2Bytes(tosend, accumCurrent);
                  swSer.write(tosend, 4);
                  lastCommShutdown = 0;
                  break;
      //PackTemp
      case 0x06:  float2Bytes(tosend, packTemp);
                  swSer.write(tosend, 4);
                  lastCommShutdown = 0;
                  break;
      //LastError
      case 0x07:  swSer.write((byte)EEPROM.read(EE_LASTERR));
                  lastCommShutdown = 0;
                  break;
      //ShutOff
      case 0x08:  swSer.write(0xAB);
                  if(lastCommShutdown)
                  {
                    disableFets();
                  }
                  lastCommShutdown = 1;
                  break;
      default:    lastCommShutdown = 0;
                  break;
    }
  }
}
