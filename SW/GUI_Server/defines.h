#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>

#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include <Adafruit_PWMServoDriver.h>

#define P_RX D6
#define P_TX D7

#define P_LED_ONBOARD D4

#define CC_READY 7
#define XREADY   5
#define OVRD     4
#define S_UV     3
#define S_OV     2
#define S_SCD    1
#define S_OCD    0
#define NO_ERR  -1


#define MCP_GPA0 0
#define MCP_GPA1 1
#define MCP_GPA2 2
#define MCP_GPA3 3
#define MCP_GPA4 4
#define MCP_GPA5 5
#define MCP_GPA6 6
#define MCP_GPA7 7
#define MCP_GPB0 8
#define MCP_GPB1 9
#define MCP_GPB2 10
#define MCP_GPB3 11
#define MCP_GPB4 12
#define MCP_GPB5 13
#define MCP_GPB6 14
#define MCP_GPB7 15

#define MCP_SW1 MCP_GPB0
#define MCP_SW2 MCP_GPB1
#define MCP_SW3 MCP_GPB2
#define MCP_SW4 MCP_GPB3
#define MCP_EN12_OB MCP_GPB4
#define MCP_VIC1 MCP_GPB5
#define MCP_VIC2 MCP_GPB6
#define MCP_EN12_EXT MCP_GPB7

#define MCP_RF0 MCP_GPA7
#define MCP_RF1 MCP_GPA6
#define MCP_RF2 MCP_GPA5
#define MCP_RF3 MCP_GPA4

#define PCA_LED0 0
#define PCA_LED1 1
#define PCA_LED2 2
#define PCA_LED3 3
#define PCA_LED4 4
#define PCA_LED5 5
#define PCA_LED6 6
#define PCA_LED7 7

#define PCA_FAN0 8
#define PCA_FAN1 9
