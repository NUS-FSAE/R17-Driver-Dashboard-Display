# R17-Driver-Dashboard-Display
NUS FSAE R17 display featuring a row of RGB shift lights and 7 segment displays

## Getting Started
### Prerequisites
1) Arduino Due
2) NUS FSAE R17 Driver Display Module

### The module utilizes serveral ICs for driving the display elements
```
TLC5971 - Texas Instruments RGB LED 4 channel driver
MAX7221 - Maxim Integrated Quad 7 segment multiplexing driver
sn7447 - BCD decoder for 1 seven segment 
```

### How it works
The Arduino Due has built in CAN controller hardware, it will use the CAN tranceiver on the custom R17 driver display module to communicate with the ECU through the CAN bus.
The code simply listens for messages in a predifined format and display it accordingly

## Tweaking the code to adapt to different CAN IDs and formats
The following functions are responsible to ONLY LISTEN TO CAN MESSAGES 
```
int listenForMessage(int identifier);
bool listenForRpmAndTp(uint16_t& rpm, float& tp);
bool listenForvehicleSpeed(float& vehicleSpeed);
void checkForWarning();
```

The following functions ONLY DRIVES THE DISPLAY with the given inputs
```
void displayRPM(uint16_t rpm);
void displayVehicleSpeed(float vehicleSpeed);
void displayThrottlePosition(float tp);
void displayGear(uint8_t& gear);
void displayRpmLeds(uint16_t rpm);
void assertWarning(int warning, bool on);
void displayFuelandCoolantTempGauge(unsigned int rpm, float fuelUsed, uint8_t coolantTemp);
```

# Modify the functions that listen to the CAN bus to suit your needs, Apologies for the bad coding style! This is due to limitations of the arduino
