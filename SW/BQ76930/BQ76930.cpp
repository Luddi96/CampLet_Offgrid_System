#include "BQ76930.h"

int32_t adcoffset = 0;
uint16_t adcgain = 0;
long rawCellVoltages[16];
float cellVoltages[7];
float packVoltage;
float packCurrent = 0;
float accumCurrent = 0;
float packTemp = 0;
unsigned long lastCheck = 0;
bool errFlag = 0;

#ifdef BQ769X0_DEBUG
  const char *byte2char(int x)
  {
    static char b[9];
    b[0] = '\0';
    int z;
    for (z = 128; z > 0; z >>= 1) strcat(b, ((x & z) == z) ? "1" : "0");
    return b;
  }
#endif

// CRC calculation taken from LibreSolar mbed firmware
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else data <<= 1;
  }

  return data;
}

void writeReg(byte address, int data)
{
  LOG_PRINT("write: ");
  LOG_PRINT(byte2char(address));
  LOG_PRINT(" --> ");
  LOG_PRINT(byte2char(data));
  uint8_t crc = 0;
  char buf[3];
  buf[0] = (char) address;
  buf[1] = data;

  // note that writes to the bq769x0 IC are: 1) start - 2) address - 3) address - 4) data - 5) CRC8 - 6) stop bit
  Wire.beginTransmission(BQ_ADD); // writes start bit - the first step
  Wire.write(buf[0]);                 // writes register address
  Wire.write(buf[1]);                 // writes data - the fourth step
 
  if (CRC_EN == true) {
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    crc = _crc8_ccitt_update(crc, (BQ_ADD << 1) | 0);
    crc = _crc8_ccitt_update(crc, buf[0]);
    crc = _crc8_ccitt_update(crc, buf[1]);
    buf[2] = crc;

    Wire.write(buf[2]); // writes CRC
    LOG_PRINT(" CRC:");
    LOG_PRINT(byte2char(buf[2]));
  }

  Wire.endTransmission();
  LOG_PRINTLN();
}

int readReg(byte address)
{  
  Wire.beginTransmission(BQ_ADD);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BQ_ADD, 1);
  return Wire.read();
}

void initBQ()
{
  //Check connection
 startcomm:
  writeReg(CC_CFG, CC_STATIC);
  LOG_PRINTLN("");
  LOG_PRINT("CC_CFG_REG: ");
  LOG_PRINTLN(byte2char(readReg(CC_CFG)));
  
  if(readReg(CC_CFG) == CC_STATIC)
  {
    LOG_PRINTLN("Comm started");
    blinkOk();
  }
  else
  {
    LOG_PRINTLN("Comm error");
    blinkCommErr();
    goto startcomm;
  }

  //Set Registers - see regmap.h
  byte towrite = 0;

  //SYSCTRL1
  towrite = (ADC_EN << 4) | (TEMP_SEL << 3);
  writeReg(SYS_CTRL1, towrite);
  //SYSCTRL2
  byte reg = readReg(SYS_CTRL2);
  towrite = (DELAY_DIS << 7) | (CC_EN << 6) | (CC_ONESHOT << 5) | (reg & 0b00000011);
  writeReg(SYS_CTRL2, towrite);
  //PROTECT1
  towrite = (RSNS << 7) | (SCD_D << 3) | (SCD_T);
  writeReg(PROTECT1, towrite);
  //PROTECT2
  towrite = (OCD_D << 4) | (OCD_T);
  writeReg(PROTECT2, towrite);
  //PROTECT3
  towrite = (UV_D << 6) | (OV_D << 4);
  writeReg(PROTECT3, towrite);

  byte gain_hi = readReg(ADCGAIN1);
  byte gain_lo = readReg(ADCGAIN2);
  int8_t offs = readReg(ADCOFFSET);

  LOG_PRINTLN("ADC Gain:");
  LOG_PRINTLN(byte2char((gain_hi)));
  LOG_PRINTLN(byte2char((gain_lo)));

  adcgain = 365 + (((gain_hi & ADCGAIN1_BITMAP) << 1) | ((gain_lo & ADCGAIN2_BITMAP) >> 5));
  LOG_PRINTLN(adcgain);

  adcoffset = 1000 * (int32_t)offs;
  LOG_PRINTLN("ADC Offs");
  LOG_PRINTLN(byte2char((offs)));
  LOG_PRINTLN(adcoffset);

  LOG_PRINTLN("High and Low cutoff:");
  uint16_t ov = 0b0010000000001000 | (adcFromVoltage(OV_MICROVOLT) & 0b0000111111110000);
  uint16_t uv = 0b0001000000000000 | (adcFromVoltage(UV_MICROVOLT) & 0b0000111111110000);
  LOG_PRINT(ov);
  LOG_PRINT(" -> ");
  LOG_PRINTLN(voltageFromAdc(ov));
  LOG_PRINT(uv);
  LOG_PRINT(" -> ");
  LOG_PRINTLN(voltageFromAdc(uv));

  towrite = ov >> 4;
  writeReg(OV_TRIP, towrite);
  towrite = uv >> 4;
  writeReg(UV_TRIP, towrite);

  attachInterrupt(digitalPinToInterrupt(P_ALERT), BQINT, RISING);
}

long voltageFromAdc(uint16_t adc)
{
  long vz = (long)adc * (long)adcgain + (long)adcoffset;
  return vz;
}
uint16_t adcFromVoltage(long voltage)
{
  long vz = voltage - adcoffset;
  vz /= adcgain;
  return (uint16_t)vz;
}
void measureCells()
{
  int k = 0;
  for(int i = 0x0C; i < 0x29;)
  {
    uint16_t adc = ((readReg(i) & 0b00111111) << 8) + (readReg(i+1));
    rawCellVoltages[k] = voltageFromAdc(adc);
    k++;
    i+=2;
  }
  mapCells();
}

void measurePack()
{  
  long pack = (readReg(BAT_HI) << 8) + (readReg(BAT_LO));
  packVoltage = (4.0 * adcgain * pack + (7*adcoffset))/1000000.0;
}

void measureTemp()
{
  float tmp_k = 0;
  int adcVal = 0;
  int vtsx = 0;
  unsigned long rts = 0;

  adcVal = ((readReg(TS2_HI) & B00111111) << 8) | readReg(TS2_LO);
  vtsx = adcVal * 0.382; // mV
  rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm

  packTemp = (1.0/(1.0/(273.15+25) + 1.0/3470.0*log(rts/10000.0)))- 273.15;

  if(packTemp < MINTEMP || packTemp > MAXTEMP)
  {
    disableFets();
  }
}

void mapCells()
{
  cellVoltages[0] = rawCellVoltages[0] / 1000000.0;
  cellVoltages[1] = rawCellVoltages[1] / 1000000.0;
  cellVoltages[2] = rawCellVoltages[2] / 1000000.0;
  cellVoltages[3] = rawCellVoltages[4] / 1000000.0;
  cellVoltages[4] = rawCellVoltages[5] / 1000000.0;
  cellVoltages[5] = rawCellVoltages[6] / 1000000.0;
  cellVoltages[6] = rawCellVoltages[9] / 1000000.0;
}

int checkStatus()
{
  byte stat = readReg(SYS_STAT);
  
  //LOG_PRINT("Status: ");
  //LOG_PRINTLN(byte2char((stat)));

  //CC
  if(stat & 0b10000000)
  {
    doCC();
  }
  
  //Check if anything else besides CC is set
  if(stat & 0b00111111)
  {
    //Find out which one
    //XREADY
    if(stat & 0b00100000)
    {
      return XREADY;
    }
    //OVRD
    if(stat & 0b00010000)
    {
      return OVRD;
    }
    //UV
    if(stat & 0b00001000)
    {
      return S_UV;
    }
    //OV
    if(stat & 0b00000100)
    {
      return S_OV;
    }
    //SCD
    if(stat & 0b00000010)
    {
      return S_SCD;
    }
    //OCD
    if(stat & 0b00000001)
    {
      return S_OCD;
    }
  }
  return NO_ERR;
}

void doCC()
{
  int16_t adc = (readReg(CC_HI) << 8) | readReg(CC_LO);
  packCurrent = ((float)adc * 8.44) / R_SNS_MOHM / 1000.0;
  if(packCurrent > -0.01 && packCurrent < 0.01)
  {
    packCurrent = 0;
  }
  accumCurrent += (packCurrent / (4*60*60));
  writeReg(SYS_STAT, 0b10000000);
}

void BQINT()
{
  //digitalWrite(P_LED_OK, HIGH);
  lastCheck = millis();
  errFlag = 1;
  //digitalWrite(P_LED_OK, LOW);
}

bool enableFets()
{
  byte stat = readReg(SYS_STAT);
  if(stat & 0b00111111)
  {
    return false;
  }
  byte reg = readReg(SYS_CTRL2);
  writeReg(SYS_CTRL2, reg | 0b00000011);
  return true;
}

void disableFets()
{
  byte reg = readReg(SYS_CTRL2);
  writeReg(SYS_CTRL2, reg & 0b11111100);
}

void float2Bytes(byte bytes_temp[4],float float_variable)
{ 
  memcpy(bytes_temp, (unsigned char*) (&float_variable), 4);
}
void bytes2Float(float* float_variable,byte bytes_temp[4])
{ 
  //*float_variable = 10.0;
  memcpy(float_variable, bytes_temp,  4);
}
