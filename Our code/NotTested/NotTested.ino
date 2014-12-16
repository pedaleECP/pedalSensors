#include "EEPROM.h"
#include "Adafruit_GFX.h"    // Core graphics library
#include "Adafruit_ST7735.h" // Hardware-specific library
#include <SPI.h>

const int LED            =  3;
const int FOOTSWITCH     =  7;
const int TOGGLE         =  2;
const int SAVE_BUTTON    = 12;
const int DISPLAY_BUTTON =  4;
const int TFT_CS         = 10;
const int TFT_RST        =  8;
const int TFT_DC         =  9;

const int DEBOUNCE_DELAY = 50;

const int MIN =     0;
const int MAX =  4096;

const int MIN_SCREEN =   0;
const int MAX_SCREEN = 115;

const int MIN_POT     =        0;
const int MAX_POT_MOD =     4096;
const int MAX_POT     = 4 * 4096;

const int MIN_SENSOR =   0;
const int MAX_SENSOR = 180;



////////////// Variables for effects ///////////////////////////

///// Distortion /////////////////////
int upper_threshold, lower_threshold;


///////////////////////////////////////////////////////////////

int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int out_DAC0, out_DAC1;

int pot0, pot1, pot2;
int pot0_mod_old = 0, pot1_mod_old = 0, pot2_mod_old = 0;

enum Modes {STANDBY1_MODE, BUTTON_MODE, STANDBY2_MODE, SENSOR_MODE, FIX_MODE} footswitch_mode;
int footswitch_detect;
int footswitch_detect_old;
int footswitch_detect_older;
int fw_last_debounce_time = 0;

bool save_button_was_not_pressed = true;
int save_button_press_time = 0;
int sb_last_debounce_time = 0;

enum Effects {DISTORTION} effect;
int display_button_enabled = true;
int db_last_debounce_time = 0;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
enum ScreenInfos {MODE_CHANGE, EFFECT_CHANGE, NO_CHANGE} screenInfo = EFFECT_CHANGE;
char* infos[]={"Standby", "Button Mode", "Sensor Mode", "Fix Mode", "Error", "Distortion"};
bool newScreenInfo = true;
int infoNr = 5;
int last_update_time   = 0;
int screenInfoTime = 0;

int p0 = 0, p1 = 0, p2 = 0;
int p0_old = 0, p1_old = 0, p2_old = 0;
int p0_saved = 0, p1_saved = 0, p2_saved = 0;
 
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
  ADC->ADC_CR = 2;       // Starts ADC conversion.
  ADC->ADC_CHER = 0xFFFF; // Enable all ADC channels

  //DAC Configuration
  analogWrite(DAC0, 0); // Enables DAC0
  analogWrite(DAC1, 0); // Enables DAC0




  pinMode(FOOTSWITCH, INPUT);
  pinMode(SAVE_BUTTON, INPUT);
  // pinMode(DISPLAY_BUTTON, INPUT);
  pinMode(LED, OUTPUT);

  tft.initR(INITR_BLACKTAB);

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


  footswitch_detect       = digitalRead(FOOTSWITCH);
  footswitch_detect_old   = footswitch_detect;
  footswitch_detect_older = footswitch_detect;

  // initialize serial communication (for edge (on/off) detection)
  Serial.begin(57600);

  // initialize the XBee's serial port transmission
  Serial1.begin(57600); 

  // From XBee
  lastByte = (byte)0;
}



void loop() {
  readFootSwitch();
  updateScreen();
  // readDisplayButton();

  switch (footswitch_mode) {
    case STANDBY1_MODE:
    case STANDBY2_MODE:
      break;
      
    case BUTTON_MODE:
      readSaveButton();
      readPotentiometer();
      break;

    case SENSOR_MODE:
      readSaveButton();
      readSensor();
      break;

    case FIX_MODE:
      readSaveButton();
      break;

    default:
      Serial.println("Invalid state");
  }

  while ((ADC->ADC_ISR & 0x1CC0) != 0x1CC0); // wait for ADC 0,1,8,9,10 conversion complete.
  in_ADC0 = ADC->ADC_CDR[7];                 // read data from ADC0
  in_ADC1 = ADC->ADC_CDR[6];                 // read data from ADC1

  switch (effect) {
    case DISTORTION:
      distortion();
      break;
    default:
      Serial.println("Invalid effect");
  }
}


void distortion() {
  upper_threshold = map(p0, 0, 4095, 4095, 2047);
  lower_threshold = map(p0, 0, 4095, 0000, 2047);

  if (in_ADC0 >= upper_threshold) in_ADC0 = upper_threshold;
  else if (in_ADC0 < lower_threshold)  in_ADC0 = lower_threshold;

  if (in_ADC1 >= upper_threshold) in_ADC1 = upper_threshold;
  else if (in_ADC1 < lower_threshold)  in_ADC1 = lower_threshold;

  //adjust the volume with POT2
  out_DAC0 = map(in_ADC0, 0, 4095, 1, p2);
  out_DAC1 = map(in_ADC1, 0, 4095, 1, p2);

  //Write the DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);   //write on DAC
  dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);   //write on DAC
}


void readFootSwitch() {
  footswitch_detect = digitalRead(FOOTSWITCH);

  if (footswitch_detect != footswitch_detect_old) {
    fw_last_debounce_time = millis();
  }

  if (footswitch_detect != footswitch_detect_older && millis() - fw_last_debounce_time > DEBOUNCE_DELAY) {
    switch (footswitch_mode) {
      case STANDBY1_MODE:
        footswitch_mode = BUTTON_MODE;
        infoNr = 1;
        break;
      case BUTTON_MODE:
        footswitch_mode = STANDBY2_MODE;
        infoNr = 0;
        break;
      case STANDBY2_MODE:
        footswitch_mode = SENSOR_MODE;
        infoNr = 2;
        break;
      case SENSOR_MODE:
        footswitch_mode = STANDBY1_MODE;
        infoNr = 0;
        break;
      case FIX_MODE:
        footswitch_mode = STANDBY1_MODE;
        infoNr = 0;
        break;
    }
    
    newScreenInfo = true;
    
    footswitch_detect_older = footswitch_detect;
  }

  footswitch_detect_old = footswitch_detect;
}


void readSaveButton() {
  if (buttonIsPressed(SAVE_BUTTON, sb_last_debounce_time) == false) {

    if (save_button_was_not_pressed == false) {
      int pressTime = millis() - save_button_press_time;

      if (pressTime > 3000) {
        p0_saved = p0;
        p1_saved = p1;
        p2_saved = p2;
      }
      else {
        p0 = p0_saved;
        p1 = p1_saved;
        p2 = p2_saved;
      }
    }

    save_button_was_not_pressed = true;
  }
  else {
    if (save_button_was_not_pressed == true) {
      save_button_press_time = millis();
      save_button_was_not_pressed = false;

      footswitch_mode = FIX_MODE;
      newScreenInfo = true;
      infoNr = 4;
    }
  }
}


void readDisplayButton() {
  if (buttonIsPressed(DISPLAY_BUTTON, db_last_debounce_time) == false) {
    display_button_enabled == true;
  }
  else {
    if (display_button_enabled == true) {
      display_button_enabled == false;
      effect = (Effects)(((int)effect + 1) % 1);

      
      screenInfo = EFFECT_CHANGE;
    }
  }
}


bool buttonIsPressed(int buttonPin, int lastDebounceTime) {
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    lastDebounceTime = millis();
  }
  else if (millis() - lastDebounceTime > DEBOUNCE_DELAY) {
    return true;
  }

  return false;
}


void readPotentiometer() {
  int pot0_mod = ADC->ADC_CDR[2]; //read from pot0
  int pot1_mod = ADC->ADC_CDR[1]; //read from pot1
  int pot2_mod = ADC->ADC_CDR[0]; //read from pot2

  updatePot(&pot0, &pot0_mod_old, pot0_mod);
  updatePot(&pot1, &pot1_mod_old, pot1_mod);
  updatePot(&pot2, &pot2_mod_old, pot2_mod);

  p0 = map(pot0, MIN_POT, MAX_POT, MIN, MAX);
  p1 = map(pot1, MIN_POT, MAX_POT, MIN, MAX);
  p2 = map(pot2, MIN_POT, MAX_POT, MIN, MAX);
}


void updatePot(int *pot, int *pot_mod_old, int pot_mod) {
  int value = 0;

  if ((pot_mod - *pot_mod_old) < -0.9 * MAX_POT_MOD) {
    value = MAX_POT_MOD + pot_mod - *pot_mod_old;//+POTSENSOR
  }
  else if ((pot_mod - *pot_mod_old) > 0.9 * MAX_POT_MOD) {
    value = - MAX_POT_MOD + pot_mod - *pot_mod_old;//+POTSENSOR
  }
  else if ((pot_mod - *pot_mod_old) > 0) {
    value = pot_mod - *pot_mod_old;//+POTSENSOR
  }
  else if ((pot_mod - *pot_mod_old) < 0) {
    value = pot_mod - *pot_mod_old;//+POTSENSOR
  }

  if ((*pot + value) > MAX_POT) {
    *pot = MAX_POT;
  }
  else if ((*pot + value) < MIN_POT) {
    *pot = MIN_POT;
  }
  else {
    *pot = *pot + value;
  }

  *pot_mod_old = pot_mod;
}


void updateScreen() {
  if (millis() - last_update_time > 100) {
    last_update_time = millis();
    
    if (newScreenInfo == true) {
      tft.fillRoundRect(0, 0, 160, 16, 0, ST7735_WHITE);
      tft.setCursor(0, 0);
      tft.print(infos[infoNr]);
      screenInfoTime = millis();
      newScreenInfo = false;
    }
    else if (millis() - screenInfoTime > 3000 && infoNr < 5) {
      tft.fillRoundRect(0, 0, 160, 16, 0, ST7735_WHITE);
      tft.setCursor(0, 0);
      
      switch (effect) {
          case DISTORTION:
            infoNr = 5;
            break;
      }
        
      newScreenInfo = true;
    }
   

    if (p0_old < p0) {
      tft.fillRoundRect(MIN_SCREEN, 32, map(p0, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p0_old > p0) {
      tft.fillRoundRect(map(p0, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 32, map(p0, MIN, MAX, MAX_SCREEN, MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p0_old = p0;


    if (p1_old < p1) {
      tft.fillRoundRect(MIN_SCREEN, 64, map(p1, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p1_old > p1) {
      tft.fillRoundRect(map(p1, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 64, map(p1, MIN, MAX, MAX_SCREEN, MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p1_old = p1;

    if (p2_old < p2) {
      tft.fillRoundRect(MIN_SCREEN, 96, map(p2, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 16, 0, ST7735_BLUE);
    }
    if (p2_old > p2) {
      tft.fillRoundRect(map(p2, MIN, MAX, MIN_SCREEN, MAX_SCREEN), 96, map(p2, MIN, MAX, MAX_SCREEN, MIN_SCREEN), 16, 0, ST7735_WHITE);
    }

    p2_old = p2;
  }
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
    Serial.print(angles[0]);
    Serial.print("|");
    Serial.print(angles[1], DEC);
    Serial.print("|");
    Serial.print(angles[2], DEC);
    Serial.print("|");

    Serial.println("");

    // Writing the angles into the global variables
    p0 = map(angles[0], MIN_SENSOR, MAX_SENSOR, MIN, MAX);
    p1 = map(angles[1], MIN_SENSOR, MAX_SENSOR, MIN, MAX);
    p2 = map(angles[2], MIN_SENSOR, MAX_SENSOR, MIN, MAX);
  }
}


