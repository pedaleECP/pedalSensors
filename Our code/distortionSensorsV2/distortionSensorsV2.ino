/*********************************************************************
***********   distortion Effect for the MovingTones pedal   **********
***********      made by the MovingTones team               **********
***********              website :)                         **********
* Licensed under a Creative Commons Attribution 3.0 Unported License *
**********************************************************************
***********  effect adapted from the distortion effect by  ***********
***********       www.electrosmash.com/pedalshield          **********
***********  Based on rcarduino.blogspot.com previous work  **********
**********************************************************************/

/* To make this code compile you will need the following libraries:
        * Adafruit_ST7735
        * Adafruit_GFX
        * EEPROM.h */

//Bibliotheques ajoutes par MovingTones
      #include "EEPROM.h"
      #include <Adafruit_GFX.h>                      // Core graphics library
      #include <Adafruit_ST7735.h>                   // Hardware-specific library
      #include <SPI.h>
      
// Variables de l'effet original
 
      int in_ADC0, in_ADC1;                        //variables for 2 ADCs values (ADC0, ADC1)
      int POT0, POT1, POT2, out_DAC0, out_DAC1;    //variables for 3 pots (ADC8, ADC9, ADC10)
      const int LED = 3;                           // LED in pin 3
      const int FOOTSWITCH = 7;                    // Footswitch in pin 7
      const int TOGGLE = 2;                        // Toggle in pin 2
      int upper_threshold, lower_threshold;        // Variable for the distortion effect
      
      

 
 //Vaiables ajoutes par le groupe MovingTones
      const int SAVE_BUTTON = 1;                  //Description de la ligne de code a faire pour toutes! 
      const int TFT_CS      = 10;
      const int TFT_RST     = 8;
      const int TFT_DC      = 9;
      
      const int STANDBY_MODE = 0;
      const int BUTTON_MODE  = 1;
      const int SENSOR_MODE  = 2;
      
      const int DEBOUNCE_DELAY = 50;
      
      const int MIN =     0;
      const int MAX =  4096;
      
      const int MIN_SCREEN =   0;
      const int MAX_SCREEN = 115;
      
      const int MAX_POT   =  4096;
      const int MIN_POT   =     0;
      const int LIMIT_POT = 4096;

      const int MAX_SENSORS = 180;
      const int MIN_SENSORS = 0;
      
     Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
  
      int footswitch_detect;
      int footswitch_detect_last;
      int footswitch_detect_previous;
      
      int save_button_detect;
      int save_button_press_time;
      boolean save_button_pressed;
      
      int footswitch_mode = STANDBY_MODE;
      
      int last_debounce_time = 0;
      int last_update_time = 0;
      
      int p0 = 0, p1 = 0, p2 = 0;
      int p0_old, p1_old, p2_old;
      int MEMORYPOTMOD0 = 0, MEMORYPOTMOD1 = 0, MEMORYPOTMOD2 = 0;
      

  

      
      
      // Variables needed for the comunication with the ArduinoNano who is sending information in Serial Port
      
              byte incomingByte;      //Bytes that are going to be used to add them and to see if there is any header between they two.
              byte lastByte;          // This will be just the byte before incomingByte

              byte byteTemp;
              int16_t counter;
              int16_t currentInt;
              int16_t int1;
              byte byteArray[3];
              byte byte1;
              byte byte2;
              byte byte3;
              int16_t angles[3];
              int16_t mask;
      

    
    
void setup()
{
  
  //Configuration from the original effect from pedalShield
  
            //ADC Configuration
            ADC->ADC_MR |= 0x80;              // DAC in free running mode.
            ADC->ADC_CR = 2;                  // Starts ADC conversion.
            ADC->ADC_CHER = 0xFFFF;           // Enable all ADC channels
          
            //DAC Configuration
            analogWrite(DAC0,0);              // Enables DAC0
            analogWrite(DAC1,0);              // Enables DAC1
      
  // We are going to read from the FootSwitch to make our 3 modes.
  
            pinMode(FOOTSWITCH, INPUT);       // Enalbles to read from the footswitch
            // pinMode(SAVE_BUTTON, INPUT);
            pinMode(LED, OUTPUT);
            
            // record the state of the footswitch when turned on
            
/*HERE WE SHOULD DETECT IF HI IS ALEREADY IN BYPASSMODE OR NOT. AND THEN START FROM HIS ORIGINAL STATE 
*******************************************************************************************************/

            footswitch_detect_last = digitalRead(FOOTSWITCH);
            footswitch_detect_previous = footswitch_detect_last;
            
    //Screen (add description and comments)
  
            tft.initR(INITR_BLACKTAB);
            tft.fillScreen(ST7735_BLACK);
          
            tft.setRotation(-1);
            tft.setTextSize(2);
            tft.fillScreen(ST7735_WHITE);
            tft.setTextColor(ST7735_RED);
            tft.println("*Distortion*");
            tft.println("");
            tft.println("          P1");
            tft.println("");
            tft.println("          P2");
            tft.println("");
            tft.println("          P3");

      
            p0_old = p0;
            p1_old = p1;
            p2_old = p2;

  


    // Initialize the Serial ports for message transmissions
          Serial.begin(57600);                   //Inicialize the port for comunication with the PC to send messages useful for coding (Serial) 
          Serial1.begin(57600);                  //Inicialize the port for comunication with the XBee (Serial1)
        
    // From XBee
         lastByte = (byte)0;     // We inicialize the lastBythe with 0 so he can make his first comparison.

 
   
}
 
void loop()
{
  
    readFootSwitch();                                  //We read the FootWwitch State and we see if it's pressed.
    updateScreen();                                    //We upload the screen information
    // readSaveButton();
    
    //From the original effect code
  
              //Read the ADCs
              while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// wait for ADC 0, 1, 8, 9, 10 conversion complete.
              in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
              in_ADC1=ADC->ADC_CDR[6];               // read data from ADC1
            
              
              //  POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
              // POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
              // POT2=ADC->ADC_CDR[12];                 // read data from ADC10  
              
              
    //Changing the parameters values depending of the mode

              switch (footswitch_mode) {
                  case STANDBY_MODE:
                    Serial.print("Standby Mode ");
                    break;            
                  case BUTTON_MODE:
                    Serial.print("Button Mode: ");
                    readPotentiometer();
                    break;
            
                  case SENSOR_MODE:
                   Serial.print("Sensor Mode: ");
                    readSensor();
                    break;
            
                  default:
                    Serial.print("Invalid state ");
                }
  
  
    //We display the parameters values on the computer screen
                Serial.print(p0);
                Serial.print("|");
                Serial.print(p1);
                Serial.print("|");
                Serial.print(p2);
                Serial.print("|");
            
                Serial.println("");

    //Code from the original effect: This is the effect, taking the parameters p0, p1 and p2
 
              upper_threshold=map(p0,0,4095,4095,2047);
              lower_threshold=map(p0,0,4095,0000,2047);
              
              if(in_ADC0>=upper_threshold) in_ADC0=upper_threshold;
              else if(in_ADC0<lower_threshold)  in_ADC0=lower_threshold;
             
              if(in_ADC1>=upper_threshold) in_ADC1=upper_threshold;
              else if(in_ADC1<lower_threshold)  in_ADC1=lower_threshold;
             
              //adjust the volume with POT2
              out_DAC0=map(in_ADC0,0,4095,1,p2);
              out_DAC1=map(in_ADC1,0,4095,1,p2);
             
              //Write the DACs
              dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
              dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);   //write on DAC
              dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
              dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);   //write on DAC
}




void readFootSwitch() {
  footswitch_detect = digitalRead(FOOTSWITCH);

  if (footswitch_detect != footswitch_detect_last) {
    last_debounce_time = millis();
  }

  if (footswitch_detect != footswitch_detect_previous && millis() - last_debounce_time > DEBOUNCE_DELAY) {
    footswitch_mode = (footswitch_mode + 1) % 3;
    footswitch_detect_previous = footswitch_detect;
  }

  footswitch_detect_last = footswitch_detect;
}


void readSaveButton() {
  save_button_detect = digitalRead(SAVE_BUTTON);

  if (save_button_detect == HIGH) {
    last_debounce_time = millis();

    if (save_button_pressed = true) {
      if (millis() - save_button_press_time > 3000) {
        EEPROM.write(0, p0);
        EEPROM.write(1, p1);
        EEPROM.write(2, p2);
      } else {
        p0 = EEPROM.read(0);
        p1 = EEPROM.read(1);
        p2 = EEPROM.read(2);
      }

      save_button_pressed = false;
    }
  }

  if (save_button_detect == LOW && millis() - last_debounce_time > DEBOUNCE_DELAY && save_button_pressed == false) {
    save_button_press_time = millis();
    save_button_pressed = true;
  }
}

void readPotentiometer() {
  //Valeur à rajouter au paramètre
  int POTMOD0 = ADC->ADC_CDR[2]; //read from pot0
  POT0 = updatePot(POT0, MEMORYPOTMOD0, POTMOD0);
  p0 = POT0;
  MEMORYPOTMOD0 = POTMOD0;
  //Serial.println(p0);
  int POTMOD1 = ADC->ADC_CDR[1]; //read from pot1
  POT1 = updatePot(POT1, MEMORYPOTMOD1, POTMOD1);
  p1 = POT1;
  MEMORYPOTMOD1 = POTMOD1;
  int POTMOD2 = ADC->ADC_CDR[0]; //read from pot2
  POT2 = updatePot(POT2, MEMORYPOTMOD2, POTMOD2);
  p2 = POT2;
  MEMORYPOTMOD2 = POTMOD2;
}


int updatePot(int POT, int MEMORYPOTMOD, int POTMOD) {
  int VALUE = 0;
  if ((POTMOD - MEMORYPOTMOD) < -0.9 * MAX_POT) {
    VALUE = MAX_POT + POTMOD - MEMORYPOTMOD;//+POTSENSOR

    if ((POT + VALUE) > LIMIT_POT) {
      POT = LIMIT_POT;
    }
    else {
      POT = POT + VALUE ;
    }
  }
  else if ((POTMOD - MEMORYPOTMOD) > 0.9 * MAX_POT) {
    VALUE = - (MAX_POT) + POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if ((POT + VALUE) < MIN_POT) {
      POT = MIN_POT;
    }
    else {
      POT = POT + VALUE ;
    }
  }
  else if ((POTMOD - MEMORYPOTMOD) > 0) {
    VALUE = POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if ((POT + VALUE) > LIMIT_POT) {
      POT = LIMIT_POT;
    }
    else {
      POT = POT + VALUE ;
    }
  }
  else if ((POTMOD - MEMORYPOTMOD) < 0) {
    VALUE = POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if ((POT + VALUE) < MIN_POT) {
      POT = MIN_POT;
    }
    else {
      POT = POT + VALUE ;
    }
  }
  return POT;
}

//Function readSensor() used to read 3 angles if there is any message comming.

void readSensor() {
  if (Serial1.available()) {                          // We just take this in count if there is some message in the Serial buffer

          boolean header = false;                     //We consider that there is not any header recieved yet, just useful for the first comparison in the next while                
          lastByte = Serial1.read();                  //We take our first byte
      
            while (Serial1.available() && !header) {
                    incomingByte = Serial1.read();    //We take our following byte
              
                    int1 = assembleInt(lastByte, incomingByte);  //We check if between our last two bytes there is any header
              
                    //Serial.println("waiting"); // Uncomment this line to see how much it takes to clear the buffer
              
                    if (int1 == -21846) {header = true;}   //Just if we have reconized a header we notify it into our "heade" boolean variable.
                    else {lastByte = incomingByte;}        // If there is not any header we take our second byte as our first byte, to then make a comparison with the following
            }
        
           readAngles();                            //Until this point we have reconized a header. This means that the next 6 bytes are our 3 angles. We read them.
          lastByte = (byte)0;                        // I think that this line it's not necesary.
        }
}

// assemble an int from the two bytes just read into byteArray
int16_t assembleInt(byte byte1, byte byte2) {

  currentInt = (int16_t)byte1 << 8;
  mask = 0xFF;
  mask = mask & (int16_t)byte2; // mask the first 8 bit to zero, in case the byte is sign-extended

  currentInt = currentInt | mask;

  // the byte1*256 + byte2 before is wrong because byte2 had been sign-extended - have to mask the first 8 bits
  //mask = 0xFF;
  //currentInt = byte1*256 + (byte2 & mask);
  return currentInt;
}

  //  Function to readthe Angles form the next 6 bytes (2 bytes for each angle)
      void readAngles() {

        for (int i = 0; i < 3; i++) {                          //We know they should be 3 angles
          
          /* We know that we should have a message in the buffer, (because we have reconized an header)
          but, it can happen that the pedal tries to read the angles form the Serial before the sensor
          manage to send him the angles (it can also be because of the data transmission speed v/s 
          the pedal's processor speed)
          so here we wait for the next message if it's necessary
          
          TODO: Use a timout!!*/  
        
                 while (!Serial1.available()) {
                    Serial.println("----Im waiting111111-");
                  }
              
          byteArray[0] = Serial1.read();                        // We read the fist byte
      
      
                while (!Serial1.available()) {                        // We wait again in case it is necesary  TODO: Use a timout!!
            
                  Serial.println("----Im waiting222222-");
                }
      
          byteArray[1] = Serial1.read();                        // We read the second byte      
          angles[i] = assembleInt(byteArray[0], byteArray[1]);  //We construct the angle using both bytes
        }
      
        /*Sometimes, (and I don't know yet why.. and this is a ToDo:try to prevent this from another way)
          there are some messages that are eathier read badly or sended badly.. so we delete any message with an
          incorrect angle*/
          
        if (angles[0] >= 0 && angles[0] <= MAX_SENSORS && angles[1] >= 0 && angles[1] <= MAX_SENSORS && angles[2] >= 0 && angles[2] <= MAX_SENSORS) {
          
        // Writing the angles into the global variables
            p0 = angles[0] * 4096 / MAX_SENSORS;              //Here we scale the angle into our effect scale. here is until 4096
            p1 = angles[1] * 4096 / MAX_SENSORS;
            p2 = angles[2] * 4096 / MAX_SENSORS;
          
          
          
        }
      }

void updateScreen() {
  if (millis() - last_update_time > 100) {
    last_update_time = millis();

    if (p0_old < p0) {
      tft.fillRoundRect(MIN_SCREEN, 32, map(p0,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p0_old > p0) {
      tft.fillRoundRect(map(p0,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 32, map(p0,MIN,MAX,MAX_SCREEN,MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p0_old = p0;


    if (p1_old < p1) {
      tft.fillRoundRect(MIN_SCREEN, 64, map(p1,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p1_old > p1) {
      tft.fillRoundRect(map(p1,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 64, map(p1,MIN,MAX,MAX_SCREEN,MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p1_old = p1;

    if (p2_old < p2) {
      tft.fillRoundRect(MIN_SCREEN, 96, map(p2,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p2_old > p2) {
      tft.fillRoundRect(map(p2,MIN,MAX,MIN_SCREEN,MAX_SCREEN), 96, map(p2,MIN,MAX,MAX_SCREEN,MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p2_old = p2;
  }
}





