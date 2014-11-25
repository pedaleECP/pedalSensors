////////////////////////////////////////////////////////////
// Arduino firmware for use with FreeSixCube processing example
////////////////////////////////////////////////////////////

// The libraries included below have been adapted to work on Arduino DUE by PedalSensor team 

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


float q[4]; //hold q values

float angles[3];

// Set the FreeIMU object
FreeSixIMU_DUE my3IMU = FreeSixIMU_DUE();

void setup() {
  Serial.begin(115200);
  Wire.begin();


  delay(5);
  my3IMU.init();
  delay(5);
}


void loop() { 
  my3IMU.getQ(q);
  serialPrintFloatArr(q, 4);
  Serial.println(""); //line break

  delay(60);
}
