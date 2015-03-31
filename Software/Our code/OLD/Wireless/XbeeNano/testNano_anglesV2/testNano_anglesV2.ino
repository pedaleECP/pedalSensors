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
//        Serial.println("sending");        
      delay(5);
      sixDOF.init(); //begin the IMU
      delay(5);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  sixDOF.getEuler(angles);
  angles[0]= int(angles[0]+180);

Serial.print(angles[0]);
Serial.print(" | ");
Serial.print(angles[1]);
Serial.print(" | ");
Serial.println(angles[2]);

//sendIntegers(angles[0],angles[1],angles[2]);
    
    
    

}


void sendIntegers(int psi, int theta, int phi){
        
        psi = psi +180;
        theta = theta + 180;
        phi = phi + 180;
        char inicio = 0xAAAA;
        Serial.print(inicio);
        Serial.print(psi);
        Serial.print(theta);
        Serial.print(phi);
        char final = 0xA0A0;
        Serial.print(final);
}
