/* Code pour l'emetteur des angles d'Euler*/




#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
float angles[3];

// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();


void setup() {
        Wire.begin();
        Serial.begin(57600);
        delay(5);
        sixDOF.init(); //begin the IMU
        delay(5);
}

void loop() {
  
  
      sixDOF.getEuler(angles);
      /*
      Serial.print(angles[0]);
      Serial.print(" | ");
      Serial.print(angles[1]);
      Serial.print(" | ");
      Serial.println(angles[2]);*/

      envoyeAngles(angles[0],angles[1],angles[2]);
    
    
    

}


void envoyeAngles(int psi, int theta, int phi){
        
        psi = int(psi+180);;
        theta = int(theta + 180);
        phi = int(phi + 180);
        char inicio = 0xAAAA;
        Serial.print(inicio);
        Serial.print(psi);
        Serial.print(theta);
        Serial.print(phi);
        char final = 0xA0A0;
        Serial.print(final);
}
