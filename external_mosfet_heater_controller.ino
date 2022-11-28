#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31865.h>
// #include "libraries/Adafruit_MAX31865_Modified.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Inputs
#define ENC_1A 2 // encoder 1 channel A - interrupt assign to this pin
#define ENC_1B 4 // encoder 1 channel B
#define ENC_2A 3 // encoder 2 channel A - interrupt assign to this pin
#define ENC_2B 5 // encoder 2 channel B
#define ENC_1S 6 // encoder 1 switch - configure as input_pullup
#define ENC_2S 7 // encoder 2 switch - configure as input_pullup

// Outputs
#define CH1_PWM 9
#define CH2_PWM 8 // THIS IS NOT A PWM PIN ON THE ARDUINO! Update with board rev.B
#define CH1_LED A0
#define CH2_LED A1

// RTDs
#define RREF 430.0
#define RNOMINAL 100.0
#define CS1 A2
#define CS2 A3
// Adafruit_MAX31865_Modified max_1 = Adafruit_MAX31865_Modified(CS1);
// Adafruit_MAX31865_Modified max_2 = Adafruit_MAX31865_Modified(CS2);
Adafruit_MAX31865 max1(CS1);
Adafruit_MAX31865 max2(CS2);

// OLEDs
#define I2C_ADDR_CH1 0x3C
#define I2C_ADDR_CH2 0x3D
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 disp1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SSD1306 disp2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global Variables
unsigned long oldTime = 0;
bool oldStateSwitch1 = 0;   // variable to track push button old state
bool oldStateSwitch2 = 0;   // variable to track push button old state
bool ch1Status = 0;         // variable that toggles channel 1 heater on
bool ch2Status = 0;         // variable that toggles channel 2 heater on
int setPoint1 = 20;         // temperature set point for channel 1 - default 20 deg C
int setPoint2 = 20;         // temperature set point for channel 2 - default 20 deg C
bool oldStateCLK1 = 0;      // variable to track encoder 1 pin A old state
bool oldStateCLK2 = 0;      // variable to track encoder 2 pin A old state 
float temperature1 = 0;     // current temperature reported by PT100, channel 1
float temperature2 = 0;     // current temperature reported by PT100, channel 2
float oldError1 = 0;
float oldError2 = 0;

// Constant PID Values
const float Kp1 = 4000; 
const float Ki1 = 3;
const float Kd1 = 1;

const float Kp2 = 4000; 
const float Ki2 = 3;
const float Kd2 = 1;

void setup(){

    // Inputs
    pinMode(ENC_1A, INPUT);
    pinMode(ENC_1B, INPUT);
    pinMode(ENC_2A, INPUT);
    pinMode(ENC_2B, INPUT);
    pinMode(ENC_1S, INPUT_PULLUP);
    pinMode(ENC_2S, INPUT_PULLUP);

    // Outputs
    pinMode(CH1_PWM, OUTPUT);
    pinMode(CH2_PWM, OUTPUT);
    pinMode(CH1_LED, OUTPUT);
    pinMode(CH2_LED, OUTPUT);

    // Write Outputs Low
    analogWrite(CH1_PWM, 0);
    analogWrite(CH2_PWM, 0);
    digitalWrite(CH1_LED, LOW);
    digitalWrite(CH2_LED, LOW);
    digitalWrite(CS1, LOW);
    digitalWrite(CS2, LOW);

    // I2C
    Wire.begin();

    // RTDs
    max1.begin(MAX31865_4WIRE);
    max2.begin(MAX31865_4WIRE);
    temperature1 = max1.temperature(RNOMINAL, RREF);
    temperature2 = max2.temperature(RNOMINAL, RREF);

    // Display Init
    if(!disp_1.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_CH1)){
        Serial.println("SSD1306 allocation failed");
        while(1); // do not proceed
    };
    
    if(!disp_2.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_CH2)){
        Serial.println("SSD1306 allocation failed");
        while(1); // do not proceed
    };


    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_1A), updateEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_2A), updateEncoder2, CHANGE);

    // Update timeOld
    timeOld = millis();
}

void loop(){

    // Read Push Button States and Toggle Heater Channels On/Off
    bool stateSwitch1 = !digitalRead(ENC_1S);
    bool stateSwitch2 = !digitalRead(ENC_2S);

    // If button pressed and wasn't previously pressed, update ch1Status
    if(stateSwitch1 && !oldStateSwitch1){
        ch1Status = !ch1Status; // toggle heater state
        oldStateSwitch1 = 1; // update old state to 'pressed'
        digitalWrite(CH1_LED, ch1Status); // toggle LED to reflect if heater channel is active
    // else if button not pressed, update old state
    }else if(!stateSwitch1){
        oldStateSwitch1 = 0;
    }

    // If button pressed and wasn't previously pressed, update ch2Status state
    if(stateSwitch2 && !oldStateSwitch2){
        ch2Status = !ch2Status; // toggle heater state
        oldStateSwitch2 = 1; // update old state to 'pressed'
        digitalWrite(CH2_LED, ch2Status); // toggle LED to reflect if heater channel is active
    // else if button not pressed, update old state
    }else if(!stateSwitch2){
        oldStateSwitch2 = 0;
    }

    // Read Temperatures
    temperature1 = max1.temperature(RNOMINAL, RREF);
    temperature2 = max2.temperature(RNOMINAL, RREF);

    // Update PID Control
    float error1 = setPoint1 - temperature1;
    float error2 = setPoint2 - temperature2;

    // Proportional Contribution
    float PID_p1 = Kp1 * error1;
    float PID_p2 = Kp2 * error2;

    // Integral Contribution
    float PID_i1 = PID_i1 + (Ki1 * error1);
    float PID_i2 = PID_i2 + (Ki2 * error2);
    
    // Derivative Contribution
    unsigned long dT = ((float)millis() - (float)oldTime) / 1000;
    float PID_d1 = Kd1 * ((error1 - oldError1) / dT);
    float PID_d2 = Kd2 * ((error2 - oldError2) / dT);
    oldError1 = error1;
    oldError2 = error2;

    // Total PID Value
    float PID1 = PID_p1 + PID_i1 + PID_d1;
    float PID2 = PID_p2 + PID_i2 + PID_d2;

    // Limit Write Value Between 0-255
    PID1 = constrain(PID1, 0, 255);
    PID2 = constrain(PID2, 0, 255);

    // Write PWM Values
    if(ch1Status){
        digitalWrite(CH1_PWM, PID1); 
    }else{
        digitalWrite(CH1_PWM, 0);
    }

    if(ch2Status){
        digitalWrite(CH2_PWM, PID2); 
    }else{
        digitalWrite(CH2_PWM, 0);
    }

    // Update oldTime
    oldTime = millis();

    // Update Displays
    disp1.setCursor(10,2);
    disp1.print("T here");
    disp1.display();

    delay(10);
}

void updateEncoder1(){

    // Read current state of pin A 'CLK', channel 1
    bool currentStateCLK1 = digitalRead(ENC_1A);

    // If the state has changed, increment/decrement counter. React to only one state change to avoid double count
    if(currentStateCLK1 != oldStateCLK1 && currentStateCLK1 == 1){
        if(digitalRead(ENC_1B) != currentStateCLK1){
            // counter clockwise
            setPoint1 -= 1;
        }else{
            // clockwise
            setPoint1 += 1;
        }
    }
}

void updateEncoder2(){

    // Read current state of pin A 'CLK', channel 2
    bool currentStateCLK2 = digitalRead(ENC_2A);

    // If the state has changed, increment/decrement counter. React to only one state change to avoid double count
    if(currentStateCLK2 != oldStateCLK2 && currentStateCLK2 == 1){
        if(digitalRead(ENC_2B) != currentStateCLK2){
            // counter clockwise
            setPoint2 -= 1;
        }else{
            // clockwise
            setPoint2 += 1;
        }
    }
}