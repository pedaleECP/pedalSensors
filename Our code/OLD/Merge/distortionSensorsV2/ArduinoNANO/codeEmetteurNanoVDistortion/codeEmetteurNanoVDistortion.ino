/* Code pour l'emetteur des angles Yaw, Pitch and Roll
See http://en.wikipedia.org/wiki/Aircraft_principal_axes*/




#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
float angles[3];
float accel[3];
float accelMAX[3]={0,0,0};
// accelMAX=(floa;
 //accelMAX[1]=(float)0;
 //accelMAX[2]=(float)0;

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
  
  
      //sixDOF.getEuler(angles);
      //sixDOF.getValues(accel);
      sixDOF.getYawPitchRoll(angles);
      
      angles[0]=angles[0]+90;
      if (angles[0]>180){angles[0]=180;}
      if (angles[0]<0){angles[0]=0;}
      angles[1]=angles[1]+90;
      angles[2]=angles[2]+90;
      
      
     /* accel[0]=(accel[0]+255)*360/510;
      accel[1]=(accel[1]+255)*360/510;
      accel[2]=(accel[2]+255)*360/510;
      if (accel[0]>360){accel[0]=360;}
      if (accel[1]>360){accel[1]=360;}
      if (accel[2]>360){accel[2]=360;}
      if (accel[0]<0){accel[0]=0;}
            if (accel[1]<0){accel[1]=0;}
                  if (accel[1]<0){accel[1]=0;}*/
    /*  
      if (accel[0]>accelMAX[0]){accelMAX[0]=accel[0];}
            if (accel[1]>accelMAX[1]){accelMAX[1]=accel[1];}
                  if (accel[2]>accelMAX[2]){accelMAX[2]=accel[2];}*/
      //accel[0]=(accel[0]+270)/540*360;
      //accel[1]=(accel[0]+270)/540.0*360.0;
      //accel[2]=(accel[0]+270)/540.0*360.0;
      
    /*  Serial.print(angles[0]);
      Serial.print(" | ");
      Serial.print(angles[1]);
      Serial.print(" | ");
      Serial.print(angles[2]);
      /*
      Serial.print(" | ");
       Serial.print(accelMAX[0]);
      Serial.print(" | ");
      Serial.print(accelMAX[1]);
      Serial.print(" | ");
      Serial.print(accelMAX[2]);*/
      //Serial.println();
      
    
      envoyeAngles(angles[1],angles[0],angles[2]);    
      //Serial.println();
      
     /* Serial.print(angles[0] +180);
      Serial.print(" | ");
      Serial.print(angles[1]+180);
      Serial.print(" | ");
      Serial.print(angles[2]+180);
      Serial.print(" | ");
      Serial.print(accel[0]);
      Serial.print(" | ");
      Serial.print(accel[1]);
      Serial.print(" | ");
      Serial.print(accel[2]);
      Serial.println();*/
        
    

}


void envoyeAngles(float psi, float theta, float phi){
        
          int16_t psiInt = (int16_t)(psi);//+180);
          int16_t thetaInt = (int16_t)(theta);// + 180);
          int16_t phiInt = (int16_t)(phi);// + 180);
          
         
      

          //char tabla[1];
          //tabla[0]= 0xAA;
          Serial.write(0xAAAA/ 256);  //on envoye le message
          Serial.write(0xAAAA% 256);  //on envoye le message

          Serial.write(psiInt / 256);
          Serial.write(psiInt% 256);

          Serial.write(thetaInt / 256);
          Serial.write(thetaInt% 256);      
        
        
          Serial.write(phiInt/ 256);
          Serial.write(phiInt% 256);
          //delay(30);

}
