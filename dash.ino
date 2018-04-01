#include <SPI.h>
#include "variant.h"
#include <due_can.h>

#define CHIP_SELECT_PIN 8
#define BCD_DECODER_A_PIN 16
#define SPI_CLOCK_DIVIDER 21

#define ON 1
#define OFF 0
#define WARNING_OIL_PRESSURE 5
#define WARNING_BATTERY_VOLTAGE 4
#define WARNING_OIL_TEMPERATURE 3
#define WARNING_ENGINE_TEMPERATURE 2
#define WARNING_FUEL_PRESSURE 1
#define WARNING_GAS_PRESSURE 0


#define CAN_DASH_MODE_ID 1568
#define CAN_GAS_WARNING 1569
#define CAN_ACTUATOR_POSITION 1588
#define CAN_RPM_TP_ID 1600
#define CAN_VEHICLE_SPEED_ID 1608
#define CAN_FUEL_USED_ID 1609
#define CAN_WARNINGS_ID 1612
#define CAN_GEAR_ID 1613
#define CAN_SWITCH_ID 1614

#define TLC5971_DATA_LENGTH 12
#define TLC5971_NUMBER 3
#define TLC5971_RPM_LEVELS 12
#define TLC5971_FUEL_LEVELS 9
#define MAX_GEAR_NUMBER 7 //including neutral gear
#define BATTERY_LOW_THRESHOLD 12.0
#define FUEL_PRESSURE_LOW_THRESHOLD 300
#define OIL_PRESSURE_LOW_THRESHOLD 50
#define OIL_TEMPERATURE_HIGH_THRESHOLD 100
#define WATER_TEMPERATURE_HIGH_THRESHOLD 100

//TLC5971 registers and data
//Function Control
#define BLANK  0
#define DSPRPT  1
#define TMGRST  0
#define EXTGCK  0
#define OUTTMG  1

//Brightness Control
#define BC_BLUE 0x7D
#define BC_GREEN 0x7D
#define BC_RED 0x7D
#define BRIGHTNESS_CONTROL ((BC_BLUE<<14) | (BC_GREEN<<7) | (BC_RED)) & 0xFFFFFF

//max7221 registers and constants
//Digits
#define DIGIT_ZERO   0x01
#define DIGIT_ONE   0x02
#define DIGIT_TWO   0x03
#define DIGIT_THREE 0x04
#define DIGIT_FOUR  0x05
#define DIGIT_FIVE  0x06
#define DIGIT_SIX   0x07
#define DIGIT_SEVEN 0x08

//Registers
#define DECODE_MODE 0x09
#define INTENSITY 0x0A
#define SCAN_LIMIT  0x0B
#define SHUTDOWN  0x0C
#define DISPLAY_TEST 0x0F

//regiser data
#define MY_INTENSITY    0x0F
#define DECODE_ALL      0xFF
#define NO_SCAN_LIMIT   0x07
#define NORMAL_OPERATION  0x01
#define DISPLAY_TEST_ON   0x01
#define DISPLAY_TEST_OFF  0x00

const static uint32_t TLC5971_CONFIGURATION = ((0x25 << 26) | (OUTTMG << 25) | (EXTGCK << 24) | (TMGRST << 23) | (DSPRPT << 22) | (BLANK << 21) | (BRIGHTNESS_CONTROL)) & 0xFFFFFFFF;
const static byte TLC5971_CONFIGURATION_BYTE1 = (TLC5971_CONFIGURATION >> 24) & 0xFF;
const static byte TLC5971_CONFIGURATION_BYTE2 = (TLC5971_CONFIGURATION >> 16) & 0xFF;
const static byte TLC5971_CONFIGURATION_BYTE3 = (TLC5971_CONFIGURATION >> 8) & 0xFF;
const static byte TLC5971_CONFIGURATION_BYTE4 = TLC5971_CONFIGURATION & 0xFF;

// lookup table for autoX mode
// 9 led for 2k rpm o 10k rpm mapping
// 12 x 16 bits for greyscale control for 1 chip
// Encoding : [shift/rpm][led number from left->right][colour] 
// format : {s3B,s3G,s3R,s2B,sG,s2R,r9B,r9G,r9R,s1B,s18G,s18R},
//      {r7B,r7G,r7R,r8B,r8G,r8R,r5B,r5G,r5R,r6B,6G,r6R},
//      {r3B,r3G,r3R,r4B,r4G,r4R,r1B,r1G,r1R,r2B,2G,r2R}
const static uint16_t ledData[TLC5971_RPM_LEVELS][TLC5971_NUMBER][TLC5971_DATA_LENGTH] = {
   { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}
    }, // 0 RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}
    }, // 1K RPM - Nothing

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000}
    }, //rpm 2 on - Green - 2K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 1 on - Green - 3K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 4 on - Green - 4K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 3 on - Blue - 5K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 6 on - Blue - 6K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 5 on - Blue - 7K RPM

    { {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
      {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 8 on - Red - 8K RPM

    { {0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000},
      {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 7 on - Red - 9K RPM

    { {0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0x0000, 0x0000},
      {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000},
      {0x0000, 0xFF00, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000}
    }, //rpm 9 on - Red - 10K RPM

    { {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00},
      {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00},
      {0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0xFF00}
    } // All red - 11K
}; 

// lookup table for enduro mode
// 9 led for 2k rpm o 10k rpm mapping
// 12 x 16 bits for greyscale control for 1 chip
// Encoding : [shift/rpm][led number from left->right][colour] 
// format : {s3B,s3G,s3R,s2B,sG,s2R,r9B,r9G,r9R,s1B,s18G,s18R},
//      {r7B,r7G,r7R,r8B,r8G,r8R,r5B,r5G,r5R,r6B,6G,r6R},
//      {r3B,r3G,r3R,r4B,r4G,r4R,r1B,r1G,r1R,r2B,2G,r2R}
const static uint16_t enduro_shift_ledData[TLC5971_DATA_LENGTH] = {
  0xFF00, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0x0000, 0x0000
};

const static uint16_t enduro_fuel_ledData[TLC5971_FUEL_LEVELS][TLC5971_DATA_LENGTH] = {   
  // empty
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000},
  // 1/8 tank - 1 dim white 3 blank
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x9900, 0x9900, 0x9900, 0x0000, 0x0000, 0x0000},
  // 1/4 tank - 1 white 3 blank
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0x0000, 0x0000, 0x0000},
  // 3/8 tank - 1 white 1 dim white
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0x9900, 0x9900, 0x9900},
  // 1/2 tank - 2 white 2 blank
  {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00},
  // 5/8 tank - 2 white 1 dim white
  {0x9900, 0x9900, 0x9900, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00},
  // 6/8 tank - 3 white 1 blank
  {0xFF00, 0xFF00, 0xFF00, 0x0000, 0x0000, 0x0000, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00},
  // 7/8 tank - 3 white 1 dim white
  {0xFF00, 0xFF00, 0xFF00, 0x9900, 0x9900, 0x9900, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00},
  // Full Tank - all white
  {0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00},
};

// colour gradient from 50 -> 105 degrees
const static uint16_t enduro_coolantTemp_ledData[13][12] = {
  {0x0000,0x0000,0x0000}, // <50 degree - blank
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000}, // 50 degree - blue
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xDF00,0x1F00,0x0000,0x0000,0x0000,0x0000}, // 55 degree
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0xBF00,0x3F00,0x0000,0x0000,0x0000,0x0000}, // 60 degree
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x9F00,0x5F00,0x0000,0x0000,0x0000,0x0000}, // 65 degree
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x7F00,0x7F00,0x0000,0x7F00,0x7F00,0x0000}, // 70 degree - teal
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x5F00,0x9F00,0x0000,0x5F00,0x9F00,0x0000}, // 75 degree
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x3F00,0xBF00,0x0000,0x3F00,0xBF00,0x0000}, // 80 degree
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x1F00,0xDF00,0x0000,0x1F00,0xDF00,0x0000}, // 85 degree
  {0x0000,0xFF00,0x0000,0x0000,0x0000,0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00,0x0000}, // 90 degree - green
  {0x0000,0xAA00,0x5500,0x0000,0x0000,0x0000,0x0000,0xAA00,0x5500,0x0000,0xAA00,0x5500}, // 95 degree - dark green
  {0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00}, // 100 degree - brown
  {0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00,0x0000,0x0000,0xFF00}  // 105 degree - red
};

enum dashMode {
  autoCross,
  endurance
};

void initMax7221();
void displayRPM(uint16_t rpm);
void displayVehicleSpeed(float vehicleSpeed);
void displayThrottlePosition(float tp);
int readRPM();
void displayGear(uint8_t& gear);
void displayRpmLeds(uint16_t rpm);
void write(byte out);
void transfer(int pin, unsigned int data, bool proceed);
int listenForMessage(int identifier);
bool listenForRpmAndTp(uint16_t& rpm, float& tp);
bool listenForvehicleSpeed(float& vehicleSpeed);
void assertWarning(int warning, bool on);
void checkForWarning();
void displayCoolantTemp();
void displayFuelandCoolantTempGauge(unsigned int rpm, float fuelUsed, uint8_t coolantTemp);

CAN_FRAME message;
uint16_t rpm = 1234;
float vehicleSpeed = 1234.0;
int waterTemp = 0.0;
float tp = 0.0;
uint8_t lastGear = 2;
uint16_t lastRPM = 1234;
uint8_t gear = 2;
bool showThrottlePosition = true;
bool neutralState = false;
bool blinkState = OFF;
bool showCoolantTemp = false;
bool flag = false;
unsigned long lastTime = 0;
unsigned long lastTime2 = 0;
const unsigned long blink_time = 25;
const unsigned long showRpmAndCoolantTempInterval = 4000;
byte configuration = (byte)TLC5971_CONFIGURATION_BYTE2;
const float maxFuel = 3.0;
float lastFuelLevel;
int lastCoolantLevel;
float fuelUsed = 0;
uint8_t coolantTemp = 99;
dashMode mode = endurance;
dashMode lastMode = endurance;
bool modeChanged = false;

void setup() {
  delay(1000);
  pinMode(8, OUTPUT); // CS
  pinMode(16, OUTPUT); //A
  pinMode(17, OUTPUT); //B
  pinMode(18, OUTPUT); //C
  pinMode(19, OUTPUT); //D
  pinMode(WARNING_OIL_PRESSURE, OUTPUT);
  pinMode(WARNING_BATTERY_VOLTAGE, OUTPUT);
  pinMode(WARNING_OIL_TEMPERATURE, OUTPUT);
  pinMode(WARNING_ENGINE_TEMPERATURE, OUTPUT);
  pinMode(WARNING_FUEL_PRESSURE, OUTPUT);
  pinMode(WARNING_GAS_PRESSURE, OUTPUT);
  assertWarning(WARNING_GAS_PRESSURE,OFF);

  initMax7221();
  //TLC5971 INIT
  pinMode(A8, OUTPUT); // SDI
  pinMode(A9, OUTPUT); // SCK
  //CAN BUS INIT
  Can0.begin(CAN_BPS_1000K);
  Can0.watchFor();
}

void loop() {
  displayGear(gear);
  checkForWarning();
  if(listenForDashMode(mode)) {
    if(lastMode != mode) {
      modeChanged = true;
      lastMode = mode;
    }
  }

  if (listenForvehicleSpeed(vehicleSpeed)) {
    if(vehicleSpeed < 1) {
      showThrottlePosition = true;
    } else {
      showThrottlePosition = false;
      displayVehicleSpeed(vehicleSpeed);
    }
    
  }
  
  if (listenForRpmAndTp(rpm, tp)) {
    if(mode == autoCross) {
      if(modeChanged) {
        displayRpmLeds(0); //clear the leds when changing mode from enduro to AutoX
        modeChanged = false;
      }
      displayRpmLeds(rpm);
    }
    
    lastRPM = rpm;
    if(showThrottlePosition) {
      displayThrottlePosition(tp);
      unsigned long currentTime = millis();
      if (currentTime - lastTime2 > showRpmAndCoolantTempInterval) {
        if (showCoolantTemp) {
          flag = true;
        } else {
          flag = false;
        }
        showCoolantTemp = !showCoolantTemp;
        lastTime2 = currentTime;
      }
      if(flag) {
        displayCoolantTemp();
      } else {
        displayRPM(rpm);
      }
    } else {
        displayRPM(rpm);
    }
  }
  
  if(mode == endurance) {
    if(listenForFuelAndCoolantTemp(fuelUsed, coolantTemp)) {
      displayFuelandCoolantTempGauge(rpm, fuelUsed, coolantTemp);
    }
  }
}

void checkForWarning() {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == 1601) {
      float fuelPressure = ((message.data.byte[4]<<8)|message.data.byte[5])/10.0;
      if(fuelPressure < FUEL_PRESSURE_LOW_THRESHOLD) {
        assertWarning(WARNING_FUEL_PRESSURE, ON);
      } else {
        assertWarning(WARNING_FUEL_PRESSURE, OFF);
      }
    }
    if (message.id == 1604) {
      float oilPressure = ((message.data.byte[6]<<8)|message.data.byte[7])/10.0;
      if(oilPressure < OIL_PRESSURE_LOW_THRESHOLD) {
        assertWarning(WARNING_OIL_PRESSURE, ON);
      } else {
        assertWarning(WARNING_OIL_PRESSURE, OFF);
      }
    }
    if (message.id == 1609) {
      float voltage = message.data.byte[5] / 10.0;
      waterTemp = (message.data.byte[0]-40);
      float oilTemp = (message.data.byte[1]-40);
      if (voltage < BATTERY_LOW_THRESHOLD) {
        assertWarning(WARNING_BATTERY_VOLTAGE, ON);
      } else {
        assertWarning(WARNING_BATTERY_VOLTAGE, OFF);
      }
      if(oilTemp > OIL_TEMPERATURE_HIGH_THRESHOLD) {
        assertWarning(WARNING_OIL_TEMPERATURE, ON);
      } else {
        assertWarning(WARNING_OIL_TEMPERATURE, OFF);
      }
      if(waterTemp > WATER_TEMPERATURE_HIGH_THRESHOLD) {
        assertWarning(WARNING_ENGINE_TEMPERATURE, ON);
      } else {
        assertWarning(WARNING_ENGINE_TEMPERATURE, OFF);
      }
    }
    if (message.id == CAN_GAS_WARNING) {
      if(message.data.byte[0]) {
        assertWarning(WARNING_GAS_PRESSURE,ON);
      } else {
        assertWarning(WARNING_GAS_PRESSURE,OFF);
      }
    }
  }
}

void assertWarning(int warning, bool on) {
  if (on) {
    digitalWrite(warning, HIGH);
  } else if (!on) {
    digitalWrite(warning, LOW);
  }
}

void transfer(int pin, unsigned int data, bool proceed) {
  digitalWrite(CHIP_SELECT_PIN, LOW);
  SPI.transfer(pin, data);
  if (!proceed) {
    digitalWrite(CHIP_SELECT_PIN, HIGH);
  }
}

void displayGear(uint8_t& gear) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == CAN_SWITCH_ID) {
      BytesUnion buffer = message.data;
      if (buffer.byte[3] & 0b1) {
        neutralState = true;
      } else {
        neutralState = false;
      }
    } else if (message.id == CAN_GEAR_ID) {
      gear = message.data.byte[6] & 0x0F;
      if (gear == lastGear) {
        return;
      } else if (neutralState) {
        gear = 0;
      } else if (gear <= 0 || gear>6) {
        gear = lastGear;
        return;
      }
      lastGear = gear;
      for (int i = 0; i < 4 ; ++i) {
        if ((gear >> i) & 0b1) {
          digitalWrite(BCD_DECODER_A_PIN + i, HIGH);
        } else {
          digitalWrite(BCD_DECODER_A_PIN + i, LOW);
        }
      }
    }
  }
}

void displayRpmLeds(uint16_t rpm) {
  int i, j;
  int r = rpm / 1000;
  if (r > 11) {
    r = 11;
  }
  // turn BLINK register on if rpm is around 9000
  if (r >= 9) {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > blink_time) {
      if (!blinkState) {
        configuration = (byte)(configuration | 0x30);
      } else {
        configuration = (byte)TLC5971_CONFIGURATION_BYTE2;
      }
      blinkState = !blinkState;
      lastTime = currentTime;
    }
  } else if (r == lastRPM/1000) {
    return;
  } else {
    configuration = (byte)TLC5971_CONFIGURATION_BYTE2;
  }
  // if car is not running, turn off all leds
  if(r == 0) {
    configuration = (byte)(configuration | 0x30);
  }
  for (i = 0; i < 3; ++i) {
    write((byte)TLC5971_CONFIGURATION_BYTE1);
    write(configuration);
    write((byte)TLC5971_CONFIGURATION_BYTE3);
    write((byte)TLC5971_CONFIGURATION_BYTE4);
    for (j = 0; j < 12; ++j) {
      write((byte)((ledData[r][i][j] >> 8) & 0xFF)); //write() can only transfer 8bits at one time
      write((byte)(ledData[r][i][j] & 0xFF)); //to transfer 2 bytes, must do separately
    }

  }
}

void write(byte out) {
  shiftOut(A8, A9, MSBFIRST, out);
}

int readRpm() {
  int value = analogRead(A0);
  return value * 10;
}

void initMax7221() {
  //SPI settings
  SPI.begin(10);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  //MAX7221 settings
  transfer(10, DECODE_MODE, true);
  transfer(10, DECODE_ALL, false);
  transfer(10, SCAN_LIMIT, true);
  transfer(10, NO_SCAN_LIMIT, false);
  transfer(10, DISPLAY_TEST, true);
  transfer(10, DISPLAY_TEST_OFF, false);
  transfer(10, INTENSITY, true);
  transfer(10, MY_INTENSITY, false);
  transfer(10, SHUTDOWN, true);
  transfer(10, NORMAL_OPERATION, false);
}

void displayRPM(uint16_t rpm) {
    uint8_t firstDigit;
    uint8_t secondDigit;
    uint8_t thirdDigit;
    uint8_t fourthDigit;
  if (rpm < 10000) {
    firstDigit = rpm % 10;
    secondDigit = (rpm / 10) % 10;
    thirdDigit = (rpm / 100) % 10;
    fourthDigit = rpm / 1000;
  } else {
    fourthDigit = rpm / 10000;
    thirdDigit = (rpm / 1000) % 10 | 0b10000000; //decimal point
    secondDigit = (rpm / 100) % 10;
    firstDigit = 0xFF; //blank
  }

  // to turn off the display for the digit that = 0 after 1st digit
  if (!fourthDigit) {
    fourthDigit = 0x7F;
    if (!thirdDigit) {
      thirdDigit = 0x7F;
      if (!secondDigit) {
        secondDigit = 0x7F;
      }
    }
  }

  transfer(10, DIGIT_THREE, true);
  transfer(10, firstDigit, false);
  transfer(10, DIGIT_TWO, true);
  transfer(10, secondDigit, false);
  transfer(10, DIGIT_ONE, true);
  transfer(10, thirdDigit, false);
  transfer(10, DIGIT_ZERO, true);
  transfer(10, fourthDigit, false);
}

void displayVehicleSpeed(float vehicleSpeed) {
  uint8_t firstDigit = (((int)(vehicleSpeed * 10)) % 10);
  uint8_t secondDigit = ((int)vehicleSpeed) % 10 | 0b10000000;
  uint8_t thirdDigit = ((int)vehicleSpeed / 10) % 10;
  uint8_t fourthDigit = (int)vehicleSpeed / 100;

  // to turn off the display for the digit that = 0 after 2nd digit
  if (!fourthDigit) {
    fourthDigit = 0x7F;
    if (!thirdDigit) {
      thirdDigit = 0x7F;
    }
  }

  transfer(10, DIGIT_SEVEN, true);
  transfer(10, firstDigit, false);
  transfer(10, DIGIT_SIX, true);
  transfer(10, secondDigit, false);
  transfer(10, DIGIT_FIVE, true);
  transfer(10, thirdDigit, false);
  transfer(10, DIGIT_FOUR, true);
  transfer(10, fourthDigit, false); 
}

void displayThrottlePosition(float tp) {
  uint8_t firstDigit = (((int)(tp * 10)) % 10);
  uint8_t secondDigit = ((int)tp) % 10 | 0b10000000;
  uint8_t thirdDigit = ((int)tp / 10) % 10;
  uint8_t fourthDigit = (int)tp / 100;

  // to turn off the display for the digit that = 0 after 2nd digit
  if (!fourthDigit) {
    fourthDigit = 0x7F;
    if (!thirdDigit) {
      thirdDigit = 0x7F;
    }
  }

  transfer(10, DIGIT_SEVEN, true);
  transfer(10, firstDigit, false);
  transfer(10, DIGIT_SIX, true);
  transfer(10, secondDigit, false);
  transfer(10, DIGIT_FIVE, true);
  transfer(10, thirdDigit, false);
  transfer(10, DIGIT_FOUR, true);
  transfer(10, fourthDigit, false);
}

void displayCoolantTemp() {
  uint8_t firstDigit = waterTemp % 10;
  uint8_t secondDigit = (waterTemp / 10) % 10;
  uint8_t thirdDigit = (waterTemp / 100) % 10;
  uint8_t fourthDigit = waterTemp / 1000;

  // to turn off the display for the digit that = 0 after 1st digit
  if (!fourthDigit) {
    fourthDigit = 0x7F;
    if (!thirdDigit) {
      thirdDigit = 0x7F;
      if (!secondDigit) {
        secondDigit = 0x7F;
      }
    }
  }

  transfer(10, DIGIT_THREE, true);
  transfer(10, firstDigit, false);
  transfer(10, DIGIT_TWO, true);
  transfer(10, secondDigit, false);
  transfer(10, DIGIT_ONE, true);
  transfer(10, thirdDigit, false);
  transfer(10, DIGIT_ZERO, true);
  transfer(10, fourthDigit, false);
}

bool listenForDashMode(dashMode &mode) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == CAN_DASH_MODE_ID) {
      BytesUnion buffer = message.data;
      if(buffer.byte[0]) {
        mode = endurance;
      } else {
        mode = autoCross;
      }
      return true;
    }
    return false;
  }
}

bool listenForGasWarning(bool dangerous) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == CAN_GAS_WARNING) {
      BytesUnion buffer = message.data;
      if(buffer.byte[0]) {
        dangerous = true;
      } else {
        dangerous = false;
      }
      return true;
    }
  }
  return false;
}

bool listenForFuelAndCoolantTemp(float &fuelUsed, uint8_t &coolantTemp) {
  if (Can0.available()) {
    Can0.read(message);
    if(message.id == CAN_FUEL_USED_ID) {
      BytesUnion buffer = message.data;
      coolantTemp = message.data.byte[0]-40;
      fuelUsed = ((buffer.byte[6] << 8) | buffer.byte[7]) / 100.0;
      return true;
    }
  }
  return false;
}

bool listenForRpmAndTp(uint16_t& rpm, float & tp) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == CAN_RPM_TP_ID) {
      BytesUnion buffer = message.data;
      rpm = buffer.byte[0] << 8 | buffer.byte[1];
      tp = ((buffer.byte[6] << 8) | buffer.byte[7]) / 10.0;
      return true;
    }
  }
  return false;
}

bool listenForvehicleSpeed(float & vehicleSpeed) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == CAN_VEHICLE_SPEED_ID) {
      BytesUnion buffer = message.data;
      vehicleSpeed = ((buffer.byte[0] << 8) | buffer.byte[1]) / 10.0;
      return true;
    }
  }
  return false;
}

void displaySomeData(int id) {
  if (Can0.available()) {
    Can0.read(message);
    if (message.id == id) {
      if(id == 1609) {
       float tp = (message.data.byte[0]-40);
  uint8_t firstDigit = (((int)(tp * 10)) % 10);
  uint8_t secondDigit = ((int)tp) % 10 | 0b10000000;
  uint8_t thirdDigit = ((int)tp / 10) % 10;
  uint8_t fourthDigit = (int)tp / 100;

  // to turn off the display for the digit that = 0 after 2nd digit
  if (!fourthDigit) {
    fourthDigit = 0x7F;
    if (!thirdDigit) {
      thirdDigit = 0x7F;
    }
  }

  transfer(10, DIGIT_SEVEN, true);
  transfer(10, firstDigit, false);
  transfer(10, DIGIT_SIX, true);
  transfer(10, secondDigit, false);
  transfer(10, DIGIT_FIVE, true);
  transfer(10, thirdDigit, false);
  transfer(10, DIGIT_FOUR, true);
  transfer(10, fourthDigit, false);
      
    }
  }
}
}

void displaySomeData2(int id) {

  if (Can0.available()) {
    Can0.read(message);
    if (message.id == id) {
      float tp = ((message.data.byte[1] << 8 | message.data.byte[2]) );
      uint8_t firstDigit = (((int)(tp * 10)) % 10);
      uint8_t secondDigit = ((int)tp) % 10 | 0b10000000;
      uint8_t thirdDigit = ((int)tp / 10) % 10;
      uint8_t fourthDigit = (int)tp / 100;

  // to turn off the display for the digit that = 0 after 2nd digit
      if (!fourthDigit) {
        fourthDigit = 0x7F;
        if (!thirdDigit) {
          thirdDigit = 0x7F;
        }
      }

      transfer(10, DIGIT_THREE, true);
      transfer(10, firstDigit, false);
      transfer(10, DIGIT_TWO, true);
      transfer(10, secondDigit, false);
      transfer(10, DIGIT_ONE, true);
      transfer(10, thirdDigit, false);
      transfer(10, DIGIT_ZERO, true);
      transfer(10, fourthDigit, false);
    }
  }
}


void displayFuelandCoolantTempGauge(unsigned int rpm, float fuelUsed, uint8_t coolantTemp) {
  int r = rpm/1000;
  // calculate fuel remaining and normalize it to the lookup table index range
  float fuelRemaining = (maxFuel - fuelUsed)/maxFuel*8;
  if(fuelRemaining < 0.6 && fuelRemaining >= 0.01) {
    fuelRemaining = 0.6;  // so that 1 light will still remain lit when it rounds down to 0
  }
  int fuelLevel = round(fuelRemaining);

  int coolantLevel = 0;
  if(coolantTemp >= 50 && coolantTemp < 55) {
    coolantLevel = 1;
  } else if(coolantTemp >= 55 && coolantTemp < 60) {
    coolantLevel = 2;
  } else if(coolantTemp >= 60 && coolantTemp < 65) {
    coolantLevel = 3;
  } else if(coolantTemp >= 65 && coolantTemp < 70) {
    coolantLevel = 4;
  } else if(coolantTemp >= 70 && coolantTemp < 75) {
    coolantLevel = 5;
  } else if(coolantTemp >= 75 && coolantTemp < 80) {
    coolantLevel = 6;
  } else if(coolantTemp >= 80 && coolantTemp < 85) {
    coolantLevel = 7;
  } else if(coolantTemp >= 85 && coolantTemp < 90) {
    coolantLevel = 8;
  } else if(coolantTemp >= 90 && coolantTemp < 95) {
    coolantLevel = 9;
  } else if(coolantTemp >= 95 && coolantTemp < 100) {
    coolantLevel = 10;
  } else if(coolantTemp >= 100 && coolantTemp < 105) {
    coolantLevel = 11;
  } else if(coolantTemp >= 105) {
    coolantLevel = 12;
  } else {
    coolantLevel = 0;
  }
  if(fuelLevel == lastFuelLevel && coolantLevel == lastCoolantLevel && r == lastRPM && r < 10) {
    return; // dont waste time writing data again
  }

  if (r >= 10) {
    unsigned long currentTime = millis();
    if (currentTime - lastTime > blink_time) {
      if (!blinkState) {
        configuration = (byte)(configuration | 0x30);
      } else {
        configuration = (byte)TLC5971_CONFIGURATION_BYTE2;
      }
      blinkState = !blinkState;
      lastTime = currentTime;
    }
  } else {
    configuration = (byte)TLC5971_CONFIGURATION_BYTE2;
  }
  for (int i = 0; i < TLC5971_NUMBER; ++i) {
    write((byte)TLC5971_CONFIGURATION_BYTE1);
    write(configuration);
    write((byte)TLC5971_CONFIGURATION_BYTE3);
    write((byte)TLC5971_CONFIGURATION_BYTE4);
    for (int j = 0; j < TLC5971_DATA_LENGTH; ++j) {
      // fill ledData for fuel gauge
      if(i == 2) {  
        write((byte)((enduro_fuel_ledData[fuelLevel][j] >> 8) & 0xFF)); 
        write((byte)(enduro_fuel_ledData[fuelLevel][j] & 0xFF)); 
      }
      // fill ledData for coolantTemp gauge
      else if(i == 1) {
        write((byte)((enduro_coolantTemp_ledData[coolantLevel][j] >> 8) & 0xFF)); 
        write((byte)(enduro_coolantTemp_ledData[coolantLevel][j] & 0xFF)); 
      }
      else if(i == 0 && (j == 6 || j == 7 || j == 8)) {
        if(coolantTemp >= 105) {
          write((byte)((enduro_coolantTemp_ledData[coolantLevel][j] >> 8) & 0xFF)); 
          write((byte)(enduro_coolantTemp_ledData[coolantLevel][j] & 0xFF)); 
      } else {
          write((byte)((0x0000 >> 8) & 0xFF)); 
          write((byte)(0x0000 & 0xFF));
        }
      }
      // if need to activate shift lights
      else if(i == 0 && (j < 6 || j > 8)) {
        if(r >= 10) {
          write((byte)((enduro_shift_ledData[j] >> 8) & 0xFF)); 
          write((byte)(enduro_shift_ledData[j] & 0xFF)); 
        } else {
          write((byte)((0x0000 >> 8) & 0xFF)); 
          write((byte)(0x0000 & 0xFF));
        }
      }
    }
  }
  lastRPM = r;
  lastFuelLevel = fuelLevel;
  lastCoolantLevel = coolantLevel;
}


