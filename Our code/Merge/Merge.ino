#include "EEPROM.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>



int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
int MEMORYPOTMOD0=0, MEMORYPOTMOD1=0, MEMORYPOTMOD2=0;

const int LED         = 3;
const int FOOTSWITCH  = 7;
const int TOGGLE      = 2;
const int SAVE_BUTTON = 1;
const int TFT_CS      = 10;
const int TFT_RST     = 8;
const int TFT_DC      = 9;

const int STANDBY_MODE = 0;
const int BUTTON_MODE  = 1;
const int SENSOR_MODE  = 2;

const int DEBOUNCE_DELAY = 50;

const int MIN =   0;
const int MAX = 100;

const int MIN_SCREEN =   0;
const int MAX_SCREEN = 115;

const int MAXPOT = 4096;
const int Pot2Max = 10000;
const int Pot2Min = 0;
const int Pot2Limit = 10000;

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

int parameter0 = MAX, parameter1 = 30, parameter2 = MIN;

void setup()
{
  
  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR=2;         // Starts ADC conversion.
  ADC->ADC_CHER=0xFFFF;  // Enable all ADC channels   
  ADC->ADC_CHER = 0xFFFF;
  // ADC->ADC_CHER=0x1CC0;  // Enable ADC channels 0 and 1. 
  
  //DAC Configuration
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC0
  
  
  pinMode(FOOTSWITCH, INPUT);
  pinMode(SAVE_BUTTON, INPUT);
  pinMode(LED, OUTPUT);

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
  // tft.fillRoundRect(0, 32, 115, 16, 0, ST7735_BLUE);
  

  // record the state of the footswitch when turned on
  footswitch_detect_last = digitalRead(FOOTSWITCH);
  footswitch_detect_previous = footswitch_detect_last;

  // initialize serial communication (for edge (on/off) detection)
  Serial.begin(9600);
}



void loop() {
  //Serial.println("Wenn das nicht angezeigt wird, haben wir ein Problem");
  //checkFootSwitch();
  //updateScreen();
  //checkPotentiometer();
  // checkSaveButton();
  
  
  
  switch (footswitch_mode) {
    case STANDBY_MODE:
      Serial.println("Standby Mode ");
      break;

    case BUTTON_MODE:
      Serial.println("Button Mode ");
      break;

    case SENSOR_MODE:
      Serial.println("Sensor Mode ");
      break;

    default:
      Serial.print("Invalid state ");
  }

}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void checkFootSwitch() {
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


void checkSaveButton() {
  save_button_detect = digitalRead(SAVE_BUTTON);

  if (save_button_detect == HIGH) {
    last_debounce_time = millis();
    
    if (save_button_pressed = true) {
      if (millis() - save_button_press_time > 3000) {
	EEPROM.write(0,parameter0);
	EEPROM.write(1,parameter1);
	EEPROM.write(2,parameter2);
      } else {
	parameter0 = EEPROM.read(0);
	parameter1 = EEPROM.read(1);
	parameter2 = EEPROM.read(2);
      }

      save_button_pressed = false;
    }
  }
  
  if (save_button_detect == LOW && millis() - last_debounce_time > DEBOUNCE_DELAY && save_button_pressed == false) {
    save_button_press_time = millis();
    save_button_pressed = true;
  }
}

void checkPotentiometer() {
  
  
  in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
  in_ADC1=ADC->ADC_CDR[6];               // read data from ADC1
  
  
  //Valeur à rajouter au paramètre
  int POTMOD= ADC->ADC_CDR[10];//read from pot0
  updatePot(POT0,MEMORYPOTMOD0,POTMOD);
  POTMOD=ADC->ADC_CDR[11]; //read from pot1
  updatePot(POT1,MEMORYPOTMOD1,POTMOD);
  POTMOD=ADC->ADC_CDR[0];//read from pot2
  updatePot(POT2,MEMORYPOTMOD2,POTMOD);
  
  //read all three potentiometeres

}

void updatePot(int POT,int MEMORYPOTMOD,int POTMOD)
{  
  int VALUE = 0; 
  if ((POTMOD - MEMORYPOTMOD) < -0.9*MAXPOT) {
    VALUE = MAXPOT + POTMOD - MEMORYPOTMOD;//+POTSENSOR 
    
    if((POT +VALUE) > Pot2Max) {
       POT = Pot2Max;
   }
    else { 
       POT = POT + VALUE ;
   }
   }
  
  
  else if ((POTMOD - MEMORYPOTMOD) > 0.9*MAXPOT) {
    VALUE = - (MAXPOT) + POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if((POT +VALUE) < Pot2Min) {
       POT = Pot2Min;
   }
    else { 
       POT = POT + VALUE ;
   }
  }
  
     
  else if ((POTMOD - MEMORYPOTMOD) > 0) {
   VALUE = POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if((POT+VALUE) > Pot2Max) {
       POT = Pot2Max;
    }
     else { 
       POT = POT + VALUE ;
    }
   }
  
     
  else if ((POTMOD - MEMORYPOTMOD) <0) {
    VALUE = POTMOD - MEMORYPOTMOD;//+POTSENSOR
    if((POT + VALUE) < Pot2Min) {
       POT = Pot2Min;
    }
     else { 
       POT = POT + VALUE ;
    }
   }
   MEMORYPOTMOD=POTMOD ;
}
  

void updateScreen() {
  if (millis() - last_update_time > 100) {
    last_update_time = millis();
    
    // delete old bars
    tft.fillRoundRect(0, 32, MAX_SCREEN, 16, 0, ST7735_WHITE);
    tft.fillRoundRect(0, 64, MAX_SCREEN, 16, 0, ST7735_WHITE);
    tft.fillRoundRect(0, 96, MAX_SCREEN, 16, 0, ST7735_WHITE);
  
    // display new bars
    tft.fillRoundRect(0, 32, MAX_SCREEN*parameter0/MAX, 16, 0, ST7735_BLUE);
    tft.fillRoundRect(0, 64, MAX_SCREEN*parameter1/MAX, 16, 0, ST7735_BLUE);
    tft.fillRoundRect(0, 96, MAX_SCREEN*parameter2/MAX, 16, 0, ST7735_BLUE);  
    }
}





















