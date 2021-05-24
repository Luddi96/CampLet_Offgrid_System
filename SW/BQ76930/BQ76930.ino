#include "BQ76930.h"
#include "app.h"

void setup() 
{
  pinInit();
  initComm();
  initBQ();
  getStat:
  int stat = checkStatus();
  if( stat == NO_ERR )
  {
    blinkOk();
    enableFets();
  }
  else
  {
    EEPROM.update(EE_LASTERR, stat);
    blinkStat(stat);
    goto getStat;
  }
  /*float test = 234.12345678;
  Serial.println(test,10);
  byte ttest[4];
  float tttest = 0;
  float2Bytes(ttest, test);
  bytes2Float(&tttest, ttest);
  Serial.println(tttest,10);
  while(1) delay(100);*/
}

uint16_t loopCnt = 0;

#define LOOP_DEBUG
void loop() 
{
  if(loopCnt >= 100)
  {
    measureCells();
    measurePack();
    measureTemp();
    loopCnt = 0;
    #ifdef LOOP_DEBUG
      Serial.println("\n\nCells:");
      for(int i = 0; i < 7; i++)
      {
        Serial.print(cellVoltages[i],6);
        Serial.print(" | ");
      }
      Serial.println("\nPack: ");
      Serial.println(packVoltage,6);
      Serial.println("Curr: ");
      Serial.println(packCurrent,6);
      Serial.println("Accum: ");
      Serial.println(accumCurrent,6);
      Serial.println("Temp: ");
      Serial.println(packTemp);
    #endif
  }

  if(swSer.available())
  {
    getSerial();
  }
  
  if(errFlag)
  {
    errFlag = 0;
    digitalWrite(P_LED_OK, HIGH);
    int stat = checkStatus();
    if(stat != NO_ERR)
    {
      digitalWrite(P_LED_OK, LOW);
      while(1) blinkStat(stat);
    }
    digitalWrite(P_LED_OK, LOW);
  }
  if(millis() - lastCheck > 500) {checkStatus();}
  loopCnt++;
  delay(10);
}
