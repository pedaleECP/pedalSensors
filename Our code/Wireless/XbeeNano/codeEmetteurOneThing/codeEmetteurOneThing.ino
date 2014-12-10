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
      
     /* Serial.print(angles[0] +180);
      Serial.print(" | ");
      Serial.print(angles[1]+180);
      Serial.print(" | ");
      Serial.println(angles[2]+180);
    delay(333);*/
      envoyeAngles(angles[0],angles[1],angles[2]);
      //Serial.println();
      
      delay(2000);
      
    
    
    

}


void envoyeAngles(float psi, float theta, float phi){
        

 
 
 
 
  int16_t psiInt = (int16_t)(psi+180);
         int16_t thetaInt = (int16_t)(theta + 180);
         int16_t phiInt = (int16_t)(phi + 180);
        
       int16_t inicio = 0xAAAA//-21846;//(unsigned int16_t)43690;
       
       
       
       
       
       Serial.print(inicio);
       Serial.print(psiInt);
       Serial.print(thetaInt);
       Serial.print(phiInt);
  /*      

Serial.write(inicio);

Serial.write(psiInt / 256);
Serial.write(psiInt% 256);
       // Serial.print(psiInt);
       // Serial.print(thetaInt);
  Serial.write(thetaInt / 256);
Serial.write(thetaInt% 256);      
        
        //Serial.print(phiInt);
        
          Serial.write(phiInt/ 256);
Serial.write(phiInt% 256); 
        
       // unsigned int16_t final = 0xA0A0;
       //x Serial.print(final);*/
}
