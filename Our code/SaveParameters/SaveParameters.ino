#include "EEPROM.h"

int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
const int LED         = 3;
const int FOOTSWITCH  = 7;
const int TOGGLE      = 2;
const int SAVE_BUTTON = 1;


int footswitch_detect;
int footswitch_detect_last;
int footswitch_detect_previous;

int save_button_detect;
int save_button_press_time;
boolean save_button_pressed;

const int STANDBY_MODE = 0;
const int BUTTON_MODE  = 1;
const int SENSOR_MODE  = 2;

int footswitch_mode = STANDBY_MODE;

const int DEBOUNCE_DELAY = 50;
int last_debounce_time = 0;

int parameter0 = 0, parameter1 = 0, parameter2 = 0;

void setup()
{
  pinMode(FOOTSWITCH, INPUT);
  pinMode(SAVE_BUTTON, INPUT);
  pinMode(LED, OUTPUT);
  

  // record the state of the footswitch when turned on
  footswitch_detect_last = digitalRead(FOOTSWITCH);
  footswitch_detect_previous = footswitch_detect_last;

  // initialize serial communication (for edge (on/off) detection)
  Serial.begin(9600);

  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR = 2;       // Starts ADC conversion.
  ADC->ADC_CHER = 0x1CC0; // Enable ADC channels 0,1,8,9 and 10

  //DAC Configuration
  analogWrite(DAC0, 0); // Enables DAC0
  analogWrite(DAC1, 0); // Enables DAC1
}



void loop() {
  // Check footswitch
  footswitch_detect = digitalRead(FOOTSWITCH);
  
  if (footswitch_detect != footswitch_detect_last) {
    last_debounce_time = millis();
  }
  
  if (footswitch_detect != footswitch_detect_previous && millis() - last_debounce_time > DEBOUNCE_DELAY) {
    footswitch_mode = (footswitch_mode + 1) % 3;
    footswitch_detect_previous = footswitch_detect;
  }
  
  footswitch_detect_last = footswitch_detect;

 
  // Check save button
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


