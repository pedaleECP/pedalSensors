// Licensed under a Creative Commons Attribution 3.0 Unported License.
// Based on rcarduino.blogspot.com previous work.
// www.electrosmash.com/pedalshield

//Inclusion de notre code.

//Bibliotheques ajouts par MovingTones
      #include "EEPROM.h"
      #include <Adafruit_GFX.h>    // Core graphics library
      #include <Adafruit_ST7735.h> // Hardware-specific library
      #include <SPI.h>
 
      
      

 
 //Vaiables de MovingTones
 
    int MEMORYPOTMOD0 = 0, MEMORYPOTMOD1 = 0, MEMORYPOTMOD2 = 0;

    const int SAVE_BUTTON = 1;
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
    
    //Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

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
    
    
    byte incomingByte;
    byte byteTemp;
    int16_t counter;
    int16_t currentInt;
    int16_t int1;
    byte byteArray[3];
    byte byte1;
    byte byte2;
    byte byte3;
    byte lastByte;
    int16_t angles[3];
    int16_t mask;
    
    
void setup()
{
  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR=2;         // Starts ADC conversion.
  ADC->ADC_CHER = 0xFFFF; // Enable all ADC channels
 
  //DAC Configuration
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC1
  
  
  
  pinMode(FOOTSWITCH, INPUT);
  // pinMode(SAVE_BUTTON, INPUT);
  pinMode(LED, OUTPUT);
  
 /* 
   tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);

  tft.setRotation(-1);
  tft.setTextSize(2);
  tft.fillScreen(ST7735_WHITE);
  tft.setTextColor(ST7735_RED);
  tft.println("EFFECT NAME");
  tft.println("");
  tft.println("          P1");
  tft.println("");
  tft.println("          P2");
  tft.println("");
  tft.println("          P3");
*/

  // record the state of the footswitch when turned on
  footswitch_detect_last = digitalRead(FOOTSWITCH);
  footswitch_detect_previous = footswitch_detect_last;

  p0_old = p0;
  p1_old = p1;
  p2_old = p2;
  
  
   // initialize serial communication (for edge (on/off) detection)
  //Serial.begin(9600);

  // initialize the XBee's serial port transmission
  Serial.begin(57600);
  Serial1.begin(57600);

  // From XBee
  lastByte = (byte)0;
  
  

  
  
  
  
}
 
void loop()
{
  
  readFootSwitch();
 // updateScreen();
  // readSaveButton();
  
  
  //Read the ADCs
  while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// wait for ADC 0, 1, 8, 9, 10 conversion complete.
  in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
  in_ADC1=ADC->ADC_CDR[6];               // read data from ADC1

  
//  POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
 // POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
 // POT2=ADC->ADC_CDR[12];                 // read data from ADC10  
//Serial.println(footswitch_mode);

  switch (footswitch_mode) {
    case STANDBY_MODE:
      Serial.println("Standby Mode ");
      break;

    case BUTTON_MODE:
    //  Serial.println("Button Mode ");
      readPotentiometer();
      break;

    case SENSOR_MODE:
     // Serial.println("Sensor Mode ");
      readSensor();
      break;

    default:
      Serial.print("Invalid state ");
  }
  
  
  Serial.print(p0);
    Serial.print("|");
    Serial.print(p1);
    Serial.print("|");
    Serial.print(p2);
    Serial.print("|");

    Serial.println("");









 
 
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


void readSensor() {
  if (Serial1.available()) {

    // The general idea is that when the pedal ends its effect loop, there can be a very big buffer of bytes waiting, that are not good, so we clear it
    boolean header = false;
    lastByte = Serial1.read();

    while (Serial1.available() && !header) {
      incomingByte = Serial1.read();

      int1 = assembleInt(lastByte, incomingByte);

      //Serial.println("waiting"); // Uncomment this line to see how much it takes to clear the buffer

      if (int1 == -21846) {
        header = true;
      } else {
        lastByte = incomingByte;
      }
    }

     readAngles();
    lastByte = (byte)0;
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


void readAngles() {
  for (int i = 0; i < 3; i++) {
    while (!Serial1.available()) {

      Serial.println("----Im waiting111111-");
    }

    byteArray[0] = Serial1.read();


    while (!Serial1.available()) {

      Serial.println("----Im waiting222222-");
    }

    byteArray[1] = Serial1.read();
    angles[i] = assembleInt(byteArray[0], byteArray[1]);
  }

  if (angles[0] >= 0 && angles[0] <= 360 && angles[1] >= 0 && angles[1] <= 360 && angles[2] >= 0 && angles[2] <= 360) {


    // Here we should change the general variables!!
  /*  Serial.print(angles[0]);
    Serial.print("|");
    Serial.print(angles[1], DEC);
    Serial.print("|");
    Serial.print(angles[2], DEC);
    Serial.print("|");

    Serial.println("");*/

    // Writing the angles into the global variables
   /* int val[3];
    val[0]= angles[0] * 4096 / 360;
    val[1] = angles[1] * 4096 / 360;
    val[2] = angles[2] * 4096 / 360;
    
    return val;*/
    
        // Writing the angles into the global variables
    p0 = angles[0] * 4096 / 180;
    p1 = angles[1] * 4096 / 180;
    p2 = angles[2] * 4096 / 180;
    
    
    
  }
}





