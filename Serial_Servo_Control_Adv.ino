// Watchman Servo Control code - Graham Jessup

// This sketch receives serial information in the format <x,y,L1,L2> where the the variables (x,y,L1,L2) are degree values.
// Angle values are then adjusted with calibration values before they are sent to each servo

// This sketch is designed to work with "Adafruit 16-Channel 12-bit PWM/Servo Shield - I2C interface" on an Arduino Uno
// This sketch requires the adafruit_PWMServo library, they have a useful guide here: https://learn.adafruit.com/adafruit-16-channel-pwm-slash-servo-shield/using-the-adafruit-library

// Serial communication code is adapted from Robin2 on Arduino forum, see here for source: https://forum.arduino.cc/index.php?topic=396450.0

// Be sure to comment out any debug Serial.print statements before connecting to raspberry pi, they will cause problems!


// Serial Reading Declarations

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0;
float XFromPi = 0.0;
//float floatFromPC = 0.0;
float YFromPi = 0.0;
float L1FromPi = 0.0;
float L2FromPi = 0.0;

boolean newData = false;


//Servo Control Declarations
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Servo header position on adafruit servo hat (using positions 0-2 and 4-6)
#define X1SERVO 0 // X Servo (left-right) on right eye
#define Y1SERVO 1 // Y Servo (up-down) on right eye
#define L1SERVO 2 // Bottom Eyelid, controlled by right-side eyelid servo

#define X2SERVO 4 // X Servo (left-right) on left eye
#define Y2SERVO 5 // Y Servo (up-down) on left eye
#define L2SERVO 6 // Top Eyelid, controlled by left-side eyelid servo
#define LaserPin 2 // This pin is used to switch laser MOSFET on/off. Mosfet is used to turn laser off when eyelids are shut.

float CtsPerDeg = 2.8; // This is a measured value, "calibrated" by driving servo to 90ish degrees (from middle position) and measuring counts
int MidPtCounts = 368; // Derived from servo midpoint = 1.5ms, 4.07 microseconds per count 1500/4.07 = 368.55

//Degree Compensations to adjust mid-point of servos - Cross-Eye fixing, adjust for mechanical errors as well
// These values are obtained by using a calibration image on the raspberry pi. 
// I used the serial monitor to command the eyelids to zero position and then adjusted the zero point until the eyes and eyelids were properly adjusted (ie pointed dead center, eyes shut when <0,0,0,0> command is received)
float X1DegComp = 4;
float X2DegComp = -5;
float Y1DegComp = -7;
float Y2DegComp = -3;
float L1DegComp = -25;
float L2DegComp = -40;
float LidCloseComp = -12; // This adds extra "clamping" force to completely close the eyes
//float L1DegTrack = 10;  


void setup() {
    Serial.begin(115200);
    //Serial.println("This demo expects 4 pieces of data - 4 floating point values");
    //Serial.println("Enter data in this style <12.5, 12.0, 24.7, 13.0>  ");
    //Serial.println();
    pwm.begin();

    pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
    pinMode(LaserPin, OUTPUT);
    delay(10);
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        SetServoPosition();
//        showParsedData();
        newData = false;
    }
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    XFromPi = atof(strtokIndx); // convert this part to a float
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    YFromPi = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    L1FromPi = atof(strtokIndx);     // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    L2FromPi = atof(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    Serial.print("X ");
    Serial.println(XFromPi);
    Serial.print("Y ");
    Serial.println(YFromPi);
    Serial.print("L1 ");
    Serial.println(L1FromPi);
    Serial.print("L2 ");
    Serial.println(L2FromPi);
}
