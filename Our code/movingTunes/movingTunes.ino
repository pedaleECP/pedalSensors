/* This is the code to upload into the Arduino to enable selection among three
 * modes and displaying the current effect on the LCD screen.
 * 
 * The three modes are:
 * 1. bypass: no effect is applied by the pedal
 * 2. buttons: a pre-coded effect is applied to the sound, controlled by three parameters
 *             as read from the three 'continuous' buttons
 * 3. sensor: the pre-coded effect is applied with the three specifying paramters taken
              from three angles calculated based on information collected by the
              accelerameter and gyroscope on the sensor
 */

// libraries required by FreeSix_cube_DUE - could take out the debug thing later
// - could take out the FreeSixIMU_DUE.h later if we don't need to see the cube - but good for visualization and debugging
#include <FreeSixIMU_DUE.h>
#include <FIMU_ADXL345_DUE.h>
#include <FIMU_ITG3200_DUE.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeSixIMU_DUE.h"
#include <Wire.h>




//-------- global variables and constants from threeModeDebounce --------------------
int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
const int LED = 3;
const int FOOTSWITCH = 7;
const int TOGGLE = 2;
int upper_threshold, lower_threshold;

int footswitch_detect;
int footswitch_detect_last;
int footswitch_detect_previous;

const int STANDBY_MODE = 0;
const int BUTTON_MODE  = 1;
const int SENSOR_MODE  = 2;

int footswitch_mode = STANDBY_MODE;

const int debounceDelay = 50;
int lastDebounceTime = 0;

int par1 = 0, par2 = 0, par3 = 0;

//----------- global variable from FreeSix_cube_DUE for the motion sensor ------------
float q[4]; //hold q values

float angles[3];

// Set the FreeIMU object
FreeSixIMU_DUE my3IMU = FreeSixIMU_DUE();


//----------- global variable from FreeSix_cube.pde from the visual program ------------
import processing.serial.*;

Serial myPort; // Create object from Serial class

final String serialPort = "/dev/tty.usbmodem1411"; // replace this with your serial port. On windows you will need something like "COM1".

float [] q = new float [4];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)


// ------------------------ NEW ---------------------------
float[] parameters = new float[3]; // psi, theta, phi from Euler in degrees


void setup() {

  //-------------------------- ThreeModeDebounce ----------------------------

  // initialize the footswitch as an input
  pinMode(FOOTSWITCH, INPUT);
  
  // initialize the LED as an output
  pinMode(LED, OUTPUT);

  // record the state of the footswitch when turned on
  footswitch_detect_last = digitalRead(FOOTSWITCH);
  footswitch_detect_previous = footswitch_detect_last;

  // initialize serial communication (for edge (on/off) detection)
  Serial.begin(9600);

  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR = 2;       // Starts ADC conversion.
  ADC->ADC_CHER = 0x1CC0; // Enable ADC channels 0,1,8,9 and 10

  //DAC Configuration
  analogWrite(DAC0, 0); // Enables DAC0
  analogWrite(DAC1, 0); // Enables DAC1


  //-------------------------- FreeSix_cube_DUE ----------------------------
  Serial.begin(115200); // not sure what this is for? compatible with the Serial.begin above?
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  //-------------------------- FreeSix_cube.pde Processing code ----------------------------
  myPort = new Serial(this, serialPort, 115200); 
  delay(100);
  myPort.clear();
  myPort.write("1");



}



void loop() {

  //-------------------------- ThreeModeDebounce ----------------------------
  footswitch_detect = digitalRead(FOOTSWITCH);
  
  if (footswitch_detect != footswitch_detect_last) {
    lastDebounceTime = millis();
  }
  
  if (footswitch_detect != footswitch_detect_previous && millis() - lastDebounceTime > debounceDelay) {
    footswitch_mode = (footswitch_mode + 1) % 3;
    footswitch_detect_previous = footswitch_detect;
  }
  
  footswitch_detect_last = footswitch_detect;

  switch (footswitch_mode) {
    case STANDBY_MODE:
      Serial.println("Standby Mode ");
      break;

    case BUTTON_MODE:
      Serial.println("Button Mode ");

      // ------------ take inputs from the buttons ----------------------



      break;

    case SENSOR_MODE:
      Serial.println("Sensor Mode ");

      // ------------ take inputs from the censor -----------------------
      readQ();

      if (hq != null) { // use home quaternion
        quaternionToEuler(quatProd(hq, q), Euler);
        text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
      }
      else {
        quaternionToEuler(q, Euler);
        text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
      }

      // convert the Euler angles to degrees, which becomes the parameters to control the effects
      parameters[0] = degrees(Euler[0]); // psi
      parameters[1] = degrees(Euler[1]); // theta
      parameters[2] = degrees(Euler[2]); // phi

      Serial.println("psi = %f", parameters[0]);
      Serial.print("theta = %f", parameters[1]);
      Serial.print("phi = %f", parameters[2]);

      break;

    default:
      Serial.print("Invalid state ");
  }



  //-------------------------- FreeSix_cube_DUE ----------------------------
  // always running in the background or just in the loop - could have pauses that way
  my3IMU.getQ(q);
  serialPrintFloatArr(q, 4);
  Serial.println(""); //line break
  delay(60);





}

//------------ functions from the FreeSix_cube.pde Processing code----------------------

float decodeFloat(String inString) {
 byte [] inData = new byte[4];

 if (inString.length() == 8) {
  inData[0] = (byte) unhex(inString.substring(0, 2));
  inData[1] = (byte) unhex(inString.substring(2, 4));
  inData[2] = (byte) unhex(inString.substring(4, 6));
  inData[3] = (byte) unhex(inString.substring(6, 8));
 }

 int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
 return Float.intBitsToFloat(intbits);
}

void readQ() {
 if (myPort.available() >= 18) {
  String inputString = myPort.readStringUntil('\n');
  //print(inputString);
  if (inputString != null && inputString.length() > 0) {
   String [] inputStringArr = split(inputString, ",");
   if (inputStringArr.length >= 5) { // q1,q2,q3,q4,\r\n so we have 5 elements
    q[0] = decodeFloat(inputStringArr[0]);
    q[1] = decodeFloat(inputStringArr[1]);
    q[2] = decodeFloat(inputStringArr[2]);
    q[3] = decodeFloat(inputStringArr[3]);
   }
  }
 }
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
 euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
 euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
 euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
 float [] q = new float[4];

 q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
 q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
 q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
 q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

 return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
 float [] q = new float[4];

 float halfAngle = angle / 2.0;
 float sinHalfAngle = sin(halfAngle);
 q[0] = cos(halfAngle);
 q[1] = -axis[0] * sinHalfAngle;
 q[2] = -axis[1] * sinHalfAngle;
 q[3] = -axis[2] * sinHalfAngle;

 return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
 float [] conj = new float[4];

 conj[0] = quat[0];
 conj[1] = -quat[1];
 conj[2] = -quat[2];
 conj[3] = -quat[3];

 return conj;
}






