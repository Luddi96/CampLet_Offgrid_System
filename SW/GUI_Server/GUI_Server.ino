#include "defines.h"
SoftwareSerial swSer(P_RX, P_TX);
Adafruit_MCP23017 mcp;
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

// Replace with your network credentials
const char* ssid = "BMS123";
const char* password = "123412341234";

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

void setup()
{  
  auxState[2] = 0;
  resetValues();
  initStuff();
  initDrivers();
  setServerRoutes();
  server.begin();
}
 
void loop()
{
  getValues();
  digitalWrite(P_LED_ONBOARD, LOW);
  delay(10);
  digitalWrite(P_LED_ONBOARD, HIGH);
  delay(1000);
}

void updateDrivers()
{
  bool extNeeded = 0;
  
  //LEDs
  for(int i = 0; i < 8; i++)
  {
    pca.setPWM(i, 0, (ledState[i] * 4095) / 100);
    if(ledState[i] != 0)
    {
      extNeeded = 1;
    }
  }

  //Debug
  pca.setPWM(PCA_FAN0, 0, (ledState[0] * 4095) / 100);
  pca.setPWM(PCA_FAN1, 0, (ledState[1] * 4095) / 100);

  //Switches
  mcp.digitalWrite(MCP_SW1 ,switchState[0]);
  mcp.digitalWrite(MCP_SW2 ,switchState[1]);
  mcp.digitalWrite(MCP_SW3 ,switchState[2]);
  mcp.digitalWrite(MCP_SW4 ,switchState[3]);

  //Aux
  //external 12V
  if(extNeeded || auxState[0])
  {
    mcp.digitalWrite(MCP_EN12_EXT, LOW);
  }
  else
  {
    mcp.digitalWrite(MCP_EN12_EXT, HIGH);
  }
  //onboard 12V (Relais --and FANs--)
  if(auxState[1] || auxState[2])
  {
    mcp.digitalWrite(MCP_EN12_OB, HIGH);
  }
  else
  {
    mcp.digitalWrite(MCP_EN12_OB, LOW);
  }
  mcp.digitalWrite(MCP_VIC1, auxState[1]);
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
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

/*
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }*/

  boolean result = WiFi.softAP("DieSkandinavischen3", "FromTheOtterSide");
  if(result == true)
  {
    Serial.println("Ready");
  }
  else
  {
    Serial.println("Failed!");
  }

  // Print ESP32 Local IP Address
  //Serial.println(WiFi.localIP());
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
  mcp.digitalWrite(MCP_EN12_EXT, HIGH);
  
  mcp.pinMode(MCP_RF0, INPUT);
  mcp.pinMode(MCP_RF1, INPUT);
  mcp.pinMode(MCP_RF2, INPUT);
  mcp.pinMode(MCP_RF3, INPUT);
}
