#include "defines.h"
#include "credentials.h"
SoftwareSerial swSer(P_RX, P_TX);
Adafruit_MCP23017 mcp;
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// Replace with your network credentials
const char* ssid = STA_SSID;
const char* password = STA_PSK;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

float cellVoltages[7];
float packVoltage = 23.78;
float packCurrent = -1.23;
float accumCurrent = 0.0123;
float packTemp = 22.34;
String lastError = "NO_ERR";

uint8_t ledState[8];
uint8_t lastLedState[8];
bool switchState[4];
bool auxState[3];

bool doFlicker = 0;

bool settingsHaveChanged = 0;

float minPV = 22.4;
float maxPV = 29.4;
float minCV = 25.5;
float maxCV = 28.7;
int minT = 0;
int maxT = 60;
bool enRem = 0;
bool enChg = 0;
bool enFli = 0;

unsigned int valErrCnt = 0;

void setup()
{ 
  auxState[2] = 0;
  resetValues();
  initStuff();
  getSettings();
  initDrivers();
  setServerRoutes();
  server.begin();
}
 
void loop()
{
  getValues();
  if(enChg)
  {
    if(packVoltage < minCV)
    {
      auxState[0] = 1;
    }
    if(packVoltage > maxCV)
    {
      auxState[0] = 0;
    }
    updateDrivers();
  }
  if(packVoltage > maxPV || packVoltage < minPV || packTemp > maxT || packTemp < minT)
  {
    valErrCnt++;
  }
  else
  {
    if(valErrCnt > 0)
    {
      valErrCnt --;
    }
  }

  // ERROR HANDLER //
  if(valErrCnt > 2)
  {
    //Undervoltage
    if(packVoltage < minPV)
    {
      //Disable 220V
      auxState[1] = 0;
      //Diable Switches
      switchState[0] = 0;
      switchState[1] = 0;
      switchState[2] = 0;
      switchState[3] = 0;
      //Set Lights to 5%
      for(int i = 0; i < 8; i++)
      {
        if(ledState[i] > 5)
        {
          ledState[i] = 5;
        }
      }
      updateDrivers();
    }
    if(packVoltage > maxPV)
    {
      //Disable Charger
      auxState[0] = 0;
      updateDrivers();
    }
  }
  if(valErrCnt > 10)
  {
    logErr();
    while(1)
    {
      swSer.write(0x08);
      delay(10);
    }
  }
  digitalWrite(P_LED_ONBOARD, LOW);
  delay(10);
  digitalWrite(P_LED_ONBOARD, HIGH);
  for(int i = 0; i < 100; i++)
  {
    if(doFlicker)
    {
      flicker();
      doFlicker = 0;
    }
    if(settingsHaveChanged)
    {
      getSettings();
      settingsHaveChanged = 0;
    }
    delay(10);
  }
}

bool logErr()
{
  File file = SPIFFS.open("/onerr.dat", "w");
  if (!file) 
  {
      file.close();
      return 0;
  }
  else
  {
    file.print("PackVoltage: ");
    file.println(packVoltage);
    file.print("PackCurrent: ");
    file.println(packCurrent);
    file.print("PackTemp: ");
    file.println(packTemp);
    file.close();
    return 1;
  }
}

void updateDrivers()
{
  //bool extNeeded = 0;
  
  //LEDs
  for(int i = 0; i < 8; i++)
  {
    pca.setPWM(i, 0, (ledState[i] * 4095) / 100);
    /*if(ledState[i] != 0)
    {
      extNeeded = 1;
    }*/
  }

  //Debug
  //pca.setPWM(PCA_FAN0, 0, (ledState[0] * 4095) / 100);
  //pca.setPWM(PCA_FAN1, 0, (ledState[1] * 4095) / 100);

  //Switches
  mcp.digitalWrite(MCP_SW1 ,switchState[0]);
  mcp.digitalWrite(MCP_SW2 ,switchState[1]);
  mcp.digitalWrite(MCP_SW3 ,switchState[2]);
  mcp.digitalWrite(MCP_SW4 ,switchState[3]);

  //Aux
  //external 12V
  /*if(extNeeded || auxState[0])
  {
    mcp.digitalWrite(MCP_EN12_EXT, HIGH);
  }
  else
  {
    mcp.digitalWrite(MCP_EN12_EXT, LOW);
  }*/
  //onboard 12V (Relais --and FANs--)
  if(auxState[0] || auxState[1] || auxState[2])
  {
    mcp.digitalWrite(MCP_EN12_OB, HIGH);
  }
  else
  {
    mcp.digitalWrite(MCP_EN12_OB, LOW);
  }
  mcp.digitalWrite(MCP_VIC1, auxState[0]);
  mcp.digitalWrite(MCP_VIC2, auxState[1]);
}

void getValues()
{
  //Cell Voltages
  while(swSer.available()) {byte trash = swSer.read(); delay(10);}
  swSer.write(0x02);
  byte cellV[28];
  int i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 0;
  while(swSer.available())
  {
    cellV[i] = swSer.read();
    i++;
  }

  if(i == 28)
  {
    for(int i = 0; i < 7; i++)
    {
      bytes2Float(&cellVoltages[i],&cellV[4*i]);
    }
    /*for(int i = 0; i < 7; i++)
    {
      Serial.println(cellVoltages[i]);
    }*/
  }

  byte buff[4];
  
  //Pack Voltage
  while(swSer.available()) {swSer.read(); delay(10);}
  swSer.write(0x03);
  i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 0;
  while(swSer.available())
  {
    buff[i] = swSer.read();
    i++;
  }
  if(i == 4)
  {
    bytes2Float(&packVoltage,buff);
  }
  
  //PackCurrent
  while(swSer.available()) {swSer.read(); delay(10);}
  swSer.write(0x04);
  i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 0;
  while(swSer.available())
  {
    buff[i] = swSer.read();
    i++;
  }
  if(i == 4)
  {
    bytes2Float(&packCurrent,buff);
  }

  //AccumCurrent
  while(swSer.available()) {swSer.read(); delay(10);}
  swSer.write(0x05);
  i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 0;
  while(swSer.available())
  {
    buff[i] = swSer.read();
    i++;
  }
  if(i == 4)
  {
    bytes2Float(&accumCurrent,buff);
  }

  //packTemp
  while(swSer.available()) {swSer.read(); delay(10);}
  swSer.write(0x06);
  i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 0;
  while(swSer.available())
  {
    buff[i] = swSer.read();
    i++;
  }
  if(i == 4)
  {
    bytes2Float(&packTemp,buff);
  }

  //lastError
  while(swSer.available()) {swSer.read(); delay(10);}
  swSer.write(0x07);
  i = 0;
  while(!swSer.available() && i < 100)
  {
    delay(1);
    i++;
  }
  delay(100);
  i = 1;
  int8_t bErr = swSer.read();
  if(i == 1)
  {
    switch(bErr)
    {
      case CC_READY:  lastError = "CC_READY";
                      break;
      case XREADY:  lastError = "XREADY";
                      break;   
      case OVRD:  lastError = "OVRD";
                      break;
      case S_UV:  lastError = "UNDERVOLTAGE";
                      break;    
      case S_OV:  lastError = "OVERVOLTAGE";
                      break;
      case S_SCD:  lastError = "SHORT";
                      break;   
      case S_OCD:  lastError = "OVERCURRENT";
                      break;
      case NO_ERR: lastError = "NO_ERR";
                      break;
      default:     lastError = "READ_ERR";   
                      break;                                        
    }
  }
}

void setServerRoutes()
{
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/bs.html");
  });

  // Route for root / web page
  server.on("/bs.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/bs.html");
  });

  server.on("/sh.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/sh.html");
  });

  server.on("/se.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/se.html");
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/favicon.ico");
  });

  server.on("/se.dat", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/se.dat");
  });

  server.on("/onerr.dat", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/onerr.dat");
  });

  server.on("/bs.dat", HTTP_GET, [](AsyncWebServerRequest *request){
    String toSend = "";
    for(int i = 0; i < 7; i++)
    {
      toSend += String(cellVoltages[i],5) + ",";
    }
    toSend += String(packVoltage,5) + ",";
    toSend += String(packCurrent,5) + ",";
    toSend += String(accumCurrent,5) + ",";
    toSend += String(packTemp,2) + ",";
    toSend += lastError;
    
    request->send(200, "text/plain", toSend);
  });

  server.on("/sh.dat", HTTP_GET, [](AsyncWebServerRequest *request){
    String toSend = "";
    for(int i = 0; i < 8; i++)
    {
      toSend += String(ledState[i]) + ",";
    }
    for(int i = 0; i < 4; i++)
    {
      toSend += String(switchState[i]) + ",";
    }
    for(int i = 0; i < 2; i++)
    {
      toSend += String(auxState[i]) + ",";
    }
    
    request->send(200, "text/plain", toSend);
  });

  server.on("/setState.php", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasArg("id") && request->hasArg("state"))
    {
      int id = (request->arg("id")).toInt();
      int state = (request->arg("state")).toInt();

      if(id < 8)
      {
        if(state == 1)
        {
          if(ledState[id] < 10)
          {
            ledState[id] ++;
          }
          else
          {
            if(ledState[id] < 100)
            {
              ledState[id] += 10;
              if(ledState[id] > 100)
              {
                ledState[id] = 100;
              }
            }
          }
        }
        else
        {
          if(ledState[id] <= 10)
          {
            if(ledState[id] > 0)
            {
              ledState[id] --;
            }            
          }
          else
          {
            ledState[id] -= 10;
          }
        }
      }
      if(id >= 8 && id < 12)
      {
        if(state == 1)
        {
          switchState[id-8] = 1;
        }
        else
        {
          switchState[id-8] = 0;
        }
      }
      if(id >= 12 && id < 14)
      {
        if(state == 1)
        {
          auxState[id-12] = 1;
        }
        else
        {
          auxState[id-12] = 0;
        }
      }
    }
    request->send(200, "text/plain", "OK");
    updateDrivers();
  }); 

  server.on("/toggleState.php", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasArg("id") && request->hasArg("state"))
    {
      int id = (request->arg("id")).toInt();
      int state = (request->arg("state")).toInt();

      if(id < 8)
      {
        if(ledState[id] == 0)
        {
          ledState[id] = lastLedState[id];
        }
        else
        {
           lastLedState[id] = ledState[id];
           ledState[id] = 0;
        }
      }
      if(id >= 8 && id < 12)
      {
        if(switchState[id-8])
        {
          switchState[id-8] = 0;
        }
        else
        {
          switchState[id-8] = 1;
        }
      }
      if(id >= 12 && id < 14)
      {
        if(auxState[id-12])
        {
          auxState[id-12] = 0;
        }
        else
        {
          auxState[id-12] = 1;
        }
      }
    }
    request->send(200, "text/plain", "OK");
    updateDrivers();
  }); 

  server.on("/setSettings.php", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasArg("minPV") && request->hasArg("maxPV") && request->hasArg("minCV") && request->hasArg("maxCV") && request->hasArg("minT") && request->hasArg("maxT") && request->hasArg("enRem") && request->hasArg("enChg") && request->hasArg("enFli") && request->hasArg("password"))
    {
      if(request->arg("password") == SETTING_PW)
      {
        float t_minPV = (request->arg("minPV")).toFloat();
        float t_maxPV = (request->arg("maxPV")).toFloat();
        float t_minCV = (request->arg("minCV")).toFloat();
        float t_maxCV = (request->arg("maxCV")).toFloat();
        int t_minT = (request->arg("minT")).toInt();
        int t_maxT = (request->arg("maxT")).toInt();
        bool t_enRem = (request->arg("enRem")).toInt();
        bool t_enChg = (request->arg("enChg")).toInt();
        bool t_enFli = (request->arg("enFli")).toInt();
  
        #define DMINV 20.0
        #define DMAXV 30.0
        #define DMINT -20
        #define DMAXT 100
        if(t_minPV >= DMINV && t_minPV <= DMAXV && t_minCV >= DMINV && t_minCV <= DMAXV && t_maxPV >= DMINV && t_maxPV <= DMAXV && t_maxCV >= DMINV && t_maxCV <= DMAXV && t_minT >= DMINT && t_minT <= DMAXT && t_maxT >= DMINT && t_maxT <= DMAXT)
        {
          String toWrite = String(t_minPV,1);
          toWrite += ",";
          toWrite += String(t_maxPV,1);
          toWrite += ",";
          toWrite += String(t_minCV,1);
          toWrite += ",";
          toWrite += String(t_maxCV,1);
          toWrite += ",";
          toWrite += String(t_minT);
          toWrite += ",";
          toWrite += String(t_maxT);
          toWrite += ",";
          toWrite += String(t_enRem);
          toWrite += ",";
          toWrite += String(t_enChg);
          toWrite += ",";
          toWrite += String(t_enFli);
          
          File file = SPIFFS.open("/se.dat", "w");
   
          if (!file) 
          {
              request->send(200, "text/plain", "WriteError-General");
          }
          else
          {
            int bytesWritten = file.print(toWrite);
             
            if (bytesWritten == 0) {
                request->send(200, "text/plain", "WriteError-Bytes");
                return;
            }
            else
            {
              request->redirect("/se.html");
              settingsHaveChanged = 1;
              //if(getSettings()) request->redirect("/se.html");
              //else request->send(200, "text/plain", "getSettings Error");
            }
          }
          file.close();
        }
        else
        {
          request->send(200, "text/plain", "Val. Range Err");
        }
      }
      else
      {
        request->send(200, "text/plain", "Password Err");
      }
    }
    else
    {
      request->send(200, "text/plain", "Param. Err");
    }
  });
}

bool getSettings()
{
  startGetSettings:
  if(!SPIFFS.exists("/se.dat"))
  {
    Serial.println("SE.DAT not found");
    if(restoreSettings())
    {
      Serial.println("SE.DAT restored");
      delay(1000);
      //goto startGetSettings;
    }
    else
    {
      return 0;
    }
  }

  readSettingsFile:
  File file = SPIFFS.open("/se.dat", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return 0;
  }
  int i = 0;
  String hasRead[9];
  while (file.available() && i < 9) 
  {
    hasRead[i] = file.readStringUntil(',');
    i++;
  }
  file.close();

  float t_minPV = hasRead[0].toFloat();
  float t_maxPV = hasRead[1].toFloat();
  float t_minCV = hasRead[2].toFloat();
  float t_maxCV = hasRead[3].toFloat();
  int t_minT = hasRead[4].toInt();
  int t_maxT = hasRead[5].toInt();
  bool t_enRem = hasRead[6].toInt();
  bool t_enChg = hasRead[7].toInt();
  bool t_enFli = hasRead[8].toInt();

  #define DMINV 20.0
  #define DMAXV 30.0
  #define DMINT -20
  #define DMAXT 100
  if(t_minPV >= DMINV && t_minPV <= DMAXV && t_minCV >= DMINV && t_minCV <= DMAXV && t_maxPV >= DMINV && t_maxPV <= DMAXV && t_maxCV >= DMINV && t_maxCV <= DMAXV && t_minT >= DMINT && t_minT <= DMAXT && t_maxT >= DMINT && t_maxT <= DMAXT)
  {
    minPV = t_minPV;
    maxPV = t_maxPV;
    minCV = t_minCV;
    maxCV = t_maxCV;
    minT = t_minT;
    maxT = t_maxT;
    enRem = t_enRem;
    enChg = t_enChg;
    enFli = t_enFli;
    return 1;
  }
  else
  {
    Serial.println("Wrong values in file, restore");
    restoreSettings();
    goto readSettingsFile;
  }
}

bool restoreSettings()
{
  File file = SPIFFS.open("/se.dat", "w");
  if (!file) 
  {
      file.close();
      return 0;
  }
  else
  {
    int bytesWritten = file.print("22.4,29.4,25.2,28.7,0,60,0,0,0");
    if (bytesWritten == 0) {
        file.close();
        return 0;
    }
    else
    {
      file.close();
      return 1;
    }
  }
}

void resetValues()
{
  for(int i = 0; i < 8; i++)
  {
    ledState[i] = 0;
    lastLedState[8] = 0;
  }
  for(int i = 0; i < 4; i++)
  {
    switchState[i] = 0;
  }
  for(int i = 0; i < 2; i++)
  {
    auxState[i] = 0;
  }
}

void initStuff()
{
  Serial.begin(115200);
  swSer.begin(9600);

  pinMode(P_LED_ONBOARD, OUTPUT);
  digitalWrite(P_LED_ONBOARD, HIGH);
  // Initialize SPIFFS
  if(!SPIFFS.begin()){
    while(1)
    {
      Serial.println("An Error has occurred while mounting SPIFFS");
      delay(1000);
    }
  }


  // Connect to Wi-Fi
  int lcnt = 0;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && lcnt < 100) {
    delay(100);
    Serial.println("Connecting to WiFi..");
    lcnt ++;
  }

  boolean result = WiFi.softAP(AP_SSID, AP_PSK);
  if(result == true)
  {
    Serial.println("Ready");
  }
  else
  {
    Serial.println("Failed!");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
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

void initDrivers()
{
  mcp.begin();
  pca.begin();

  pca.setOutputMode(true);
  pca.setPWMFreq(200);
  
  mcp.pinMode(MCP_SW1, OUTPUT);
  mcp.pinMode(MCP_SW2, OUTPUT);
  mcp.pinMode(MCP_SW3, OUTPUT);
  mcp.pinMode(MCP_SW4, OUTPUT);
  mcp.pinMode(MCP_EN12_OB, OUTPUT);
  mcp.pinMode(MCP_VIC1, OUTPUT);
  mcp.pinMode(MCP_VIC2, OUTPUT);
  
  mcp.pinMode(MCP_EN12_EXT, OUTPUT);
  mcp.digitalWrite(MCP_EN12_EXT, LOW);
  
  mcp.pinMode(MCP_RF0, INPUT);
  mcp.pinMode(MCP_RF1, INPUT);
  mcp.pinMode(MCP_RF2, INPUT);
  mcp.pinMode(MCP_RF3, INPUT);

  mcp.setupInterrupts(true,false,LOW);
  mcp.setupInterruptPin(MCP_RF0, CHANGE);
  mcp.setupInterruptPin(MCP_RF1, CHANGE);
  mcp.setupInterruptPin(MCP_RF2, CHANGE);
  mcp.setupInterruptPin(MCP_RF3, CHANGE);

  attachInterrupt(digitalPinToInterrupt(P_PCINTA), rfInterrupt, FALLING);
}

unsigned long lastPress = 0;
int lastPressed = -1;

ICACHE_RAM_ATTR void rfInterrupt() 
{
  int tempLastPressed = -1;
  if(mcp.digitalRead(MCP_RF0))
  {
    tempLastPressed = 0;
  }
  if(mcp.digitalRead(MCP_RF1))
  {
    tempLastPressed = 1;
  }
  if(mcp.digitalRead(MCP_RF2))
  {
    tempLastPressed = 2;
  }
  if(mcp.digitalRead(MCP_RF3))
  {
    tempLastPressed = 3;
  }

  //One of the signals was high
  if(tempLastPressed != -1)
  {
    lastPressed = tempLastPressed;
    lastPress = millis();
  }
  //All were released
  else
  {
    unsigned long diff = millis() - lastPress;
    if(diff > 100 && diff < 3000 && lastPressed != -1)
    {
      if(diff > 100 && diff < 700)
      {
        if(enRem) doPress(lastPressed, 0);
      }
      else
      {
        if(enRem) doPress(lastPressed, 1);
      }
      lastPressed = -1;
    }
  }
}

#define STAGE1 10
#define STAGE2 50
#define STAGE3 100

int doStages(int input)
{
  if(input == 0)
  {
    return STAGE1;
  }
  else if (input > 0 && input < 10)
  {
    return STAGE1;
  }
  else if (input == 10)
  {
    return STAGE2;
  }
  else if (input > 10 && input < 50)
  {
    return STAGE2;
  }
  else if (input == 50)
  {
    return STAGE3;
  }
  else if (input > 50 && input < 100)
  {
    return STAGE3;
  }
  else if (input == 100)
  {
    return 0;
  }
}

void doPress(int chan, bool len)
{
  if(!len)
  {
    if(chan == 0)
    {
      ledState[0] = doStages(ledState[0]);
    }
    if(chan == 1)
    {
      ledState[1] = doStages(ledState[1]);
    }
    if(chan == 2)
    {
      if(ledState[2] != ledState[3])
      {
        ledState[2] = ledState[3] = min(ledState[2], ledState[3])-1;
        if(ledState[2] == 0) ledState[2] = ledState[3] = 1;
      }
      ledState[3] = ledState[2] = doStages(ledState[2]);
    }
    if(chan == 3)
    {
      if(ledState[0] == 0 && ledState[1] == 0 && ledState[2] == 0 && ledState[3] == 0 && enFli)
      {
        doFlicker = 1;
        return;
      }
      if(ledState[0] != ledState[1] || ledState[0] != ledState[2] || ledState[0] != ledState[3])
      {
        ledState[0] = ledState[1] = ledState[2] = ledState[3] = min(min(ledState[0], ledState[1]),min(ledState[2], ledState[3]));
        if(ledState[0] > 0) ledState[0] = ledState[1] = ledState[2] = ledState[3] = ledState[0] - 1;
        if(ledState[0] == 0) ledState[0] = ledState[1] = ledState[2] = ledState[3] = 1;
      }
      ledState[0] = ledState[1] = ledState[2] = ledState[3] = doStages(ledState[0]);
    }
  }
  else
  {
    if(chan == 0) ledState[0] = 0;
    if(chan == 1) ledState[1] = 0;
    if(chan == 2)
    {
      ledState[2] = 0;
      ledState[3] = 0;
    }
    if(chan == 3)
    {
      ledState[0] = 0;
      ledState[1] = 0;
      ledState[2] = 0;
      ledState[3] = 0;
    }
  }
  updateDrivers();
}

const int blinkOn[] = {10, 20, 20, 240, 20, 40, 20, 100, 20, 20, 20, 260, 80, 20, 240, 60, 160, 20, 240, 20, 1000, 20, 20, 40, 100, 20, 2740, 340, 860, 20, 500, 20, 60, 20};
void flicker()
{
  for(int i=0; i<sizeof(blinkOn)/sizeof(int); ++i) 
  {
    ledState[0] = ledState[2] = (!(1&i)) * 50;
    updateDrivers();
    delay(blinkOn[i]/2);
    ledState[1] = ledState[3] = (!(1&i)) * 50;
    updateDrivers();
    delay(blinkOn[i]/2);
  }
  ledState[0] = ledState[2] = ledState[1] = ledState[3] = 50;
  updateDrivers();
}
