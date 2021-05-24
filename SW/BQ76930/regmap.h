#define SYS_STAT 0x00
  #define CC_READY 7
  #define XREADY   5
  #define OVRD     4
  #define S_UV     3
  #define S_OV     2
  #define S_SCD    1
  #define S_OCD    0
  #define NO_ERR  -1
  
#define CB1 0x01
#define CB2 0x02
#define CB3 0x03

#define SYS_CTRL1 0x04
	//ADC Enabled
	//0x0:disabled 0x1:enabled
	#define ADC_EN 0x1

	//Select Temp Sensor
	//0x0:chip(die) 0x1:external
	#define TEMP_SEL 0x1

#define SYS_CTRL2 0x05
	//Disable OV, UV, OCD, and SCD delays
	//0x0:Normal 0x1:no_delay
	#define DELAY_DIS 0x0

	//Coulomb counter continuous operation enable
	//0x0:CC_off 0x1:CC_on
	#define CC_EN 0x1

	//CC singe reaing trigger
	//0x0:No_act 0x1:single_CC_reading
	#define CC_ONESHOT 0x0
	
#define PROTECT1 0x06
#define PROTECT2 0x07

	//Sensitivity (See tables for SCD and OCD)
	//0x0: more_sens 0x1: less_sens
	#define RSNS 0x1
	
	//Delay for SCD
	// 0x0  |   70us
	// 0x1  |  100us
	// 0x2  |  100us
	// 0x3  |  400us
	#define SCD_D 0x1
	
	//Level for SCD in mV (divide by shunt res.)
	//      | RSNS 1 | RSNS 0
	// 0x0  |   44   |   22
	// 0x1  |   67   |   33
	// 0x2  |   89   |   44
	// 0x3  |  111   |   56
	// 0x4  |  133   |   67
	// 0x5  |  155   |   78
	// 0x6  |  178   |   89
	// 0x7  |  200   |  100
	#define SCD_T 0x2
	
	//Delay for OCD
	// 0x0  |    8ms
	// 0x1  |   20ms
	// 0x2  |   40ms
	// 0x3  |   80ms
	// 0x4  |  160ms
	// 0x5  |  320ms
	// 0x6  |  640ms
	// 0x7  | 1280ms
	#define OCD_D 0x5
	
	//Level for OCD in mV (divide by shunt res.)
	//      | RSNS 1 | RSNS 0
	// 0x0  |   17   |    8
	// 0x1  |   22   |   11
	// 0x2  |   28   |   14
	// 0x3  |   33   |   17
	// 0x4  |   39   |   19
	// 0x5  |   44   |   22
	// 0x6  |   50   |   25
	// 0x7  |   56   |   28
	// 0x8  |   61   |   31
	// 0x9  |   67   |   33
	// 0xA  |   72   |   36
	// 0xB  |   78   |   39
	// 0xC  |   83   |   42
	// 0xD  |   89   |   44
	// 0xE  |   94   |   47
	// 0xF  |  100   |   50
	#define OCD_T 0x8
	
#define PROTECT3 0x08
	
	//Undervoltage Delay
	// 0x0   1s
	// 0x1   4s
	// 0x2   8s
	// 0x3  16s
	#define UV_D 0x1
	
	//Overvoltage Delay
	// 0x0   1s
	// 0x1   2s
	// 0x2   4s
	// 0x3   8s
	#define OV_D 0x1
	
#define OV_TRIP 0x09
	//Overvoltage Trip
  #define OV_MICROVOLT 4200000
	/*Middle 8 bits of the direct ADC mapping of the desired OV protection threshold, with upper
	2 MSB set to 10 and lower 2 LSB set to 1000. The equivalent OV threshold is mapped to:
	10-OV_T<7:0>1000.
	By default, OV_TRIP is configured to a 0xAC setting.
	Note that OV_TRIP is based on the ADC voltage, which requires back-calculation using the GAIN and OFFSET
	values stored in ADCGAIN<4:0>and ADCOFFSET<7:0>*/
	//Set by function

#define UV_TRIP 0x0A	
	//Undervoltage Trip
 #define UV_MICROVOLT 3000000
	/*Middle 8 bits of the direct ADC mapping of the desired UV protection threshold, with upper
	2 MSB set to 01 and lower 4 LSB set to 0000. In other words, the equivalent OV threshold is mapped to: 01-
	UV_T<7:0>–0000.
	By default, UV_TRIP is configured to a 0x97 setting. .
	Note that UV_TRIP is based on the ADC voltage, which requires back-calculation using the GAIN and OFFSET
	values stored in ADCGAIN<4:0>and ADCOFFSET<7:0>.*/
	//Set by function
	
#define CC_CFG 0x0B
	//Static, must be written
	#define CC_STATIC 0x19
	
//VC1_HI to VC15_LO will be done in function

//Voltage of whole pack
#define BAT_HI 0x2A
#define BAT_LO 0x2B

//Voltage of Temp Sens
#define TS1_HI 0x2C
#define TS1_LO 0x2D
#define TS2_HI 0x2E
#define TS2_LO 0x2F
#define TS3_HI 0x30
#define TS3_LO 0x31

//Current in CC
#define CC_HI 0x32
#define CC_LO 0x33

//ADC Gain  (trimmed from factory)
#define ADCGAIN1 0x50
#define ADCGAIN1_BITMAP 0b00001100
#define ADCGAIN2 0x59
#define ADCGAIN2_BITMAP 0b11100000

//ADC Offset (trimmed from factory)
#define ADCOFFSET 0x51
#define ADCOFFSET_BITMAP 0b00011111
