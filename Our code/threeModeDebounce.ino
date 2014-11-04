int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
const int LED = 3;
const int FOOTSWITCH = 7;
const int TOGGLE = 2;
int upper_threshold, lower_threshold;

int footswitch_detect;
int footswitch_detect_last;
int footswitch_detect_previous;

const int STANDBY_MODE = 0;
const int BUTTON_MODE  = 1;
const int SENSOR_MODE  = 2;

int footswitch_mode = STANDBY_MODE;

const int debounceDelay = 50;
int lastDebounceTime = 0;

void setup()
{
  // initialize the footswitch as an input
  pinMode(FOOTSWITCH, INPUT);
  
  // initialize the LED as an output
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


/*
Program for testing the different modes. In the standby mode the LED is switched off, in the button mode the led blinks and in the sensor mode the LED is switched on.
In the case of an indeterminate mode a text is displayed on the computer monitor.  
*/
void loop() {
  
  footswitch_detect = digitalRead(FOOTSWITCH);
  
  if (footswitch_detect != footswitch_detect_last) {
    lastDebounceTime = millis();
  }
  
  if (footswitch_detect != footswitch_detect_previous && millis() - lastDebounceTime > debounceDelay) {
    footswitch_mode = (footswitch_mode + 1) % 3;
    footswitch_detect_previous = footswitch_detect;
  }
  
  footswitch_detect_last = footswitch_detect;
  
 

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
