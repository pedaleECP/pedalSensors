#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>

float angles[3]; // yaw pitch roll

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();


void setup() {
  // put your setup code here, to run once:
  Wire.begin();

  Serial.begin(57600);
  Serial.println("sending");
  
delay(5);
sixDOF.init(); //begin the IMU
delay(5);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay (2000);
  
  
  sixDOF.getEuler(angles);

Serial.print(angles[0]);
Serial.print(" | ");
Serial.print(angles[1]);
Serial.print(" | ");
Serial.println(angles[2]);
   // float a = millis();
    //Serial.println(a);
    
    
    

}
