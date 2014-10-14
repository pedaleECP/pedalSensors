// Licensed under a Creative Commons Attribution 3.0 Unported License.
// Based on rcarduino.blogspot.com previous work.
// www.electrosmash.com/pedalshield
 
int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
const int LED = 3;
const int FOOTSWITCH = 7; 
const int TOGGLE = 2; 
int upper_threshold, lower_threshold;
int footswitch_detect = 0;
int footswitch_state = 0;
int footswitch_state_last;
 
void setup()
{
  // initialize the footswitch as an input
  pinMode(FOOTSWITCH, INPUT);
  
  // initialize the LED as an output
  pinMode(LED, OUTPUT);
  
  // record the state of the footswitch when turned on
  footswitch_state_last = digitalRead(FOOTSWITCH);
  
  // initialize serial communication (for edge (on/off) detection)
  Serial.begin(9600);
  Serial.println("In the setup loop");
  
  //ADC Configuration
  ADC->ADC_MR |= 0x80;   // DAC in free running mode.
  ADC->ADC_CR=2;         // Starts ADC conversion.
  ADC->ADC_CHER=0x1CC0;  // Enable ADC channels 0,1,8,9 and 10  
 
  //DAC Configuration
  analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC1
}
 
 
void loop()
{ 
  footswitch_detect = digitalRead(FOOTSWITCH);
  
  if (footswitch_detect != footswitch_state_last) {
    footswitch_state = (footswitch_state + 1) % 3;
    footswitch_state_last = footswitch_detect;
  }

  if (footswitch_state == 0) {
    Serial.print("In case 0 = ");
    Serial.println(footswitch_state);
    // LED blinks every 1 second
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);              // wait for a second
  }
  else if (footswitch_state == 1) {
    Serial.print("In case 1 = ");
    Serial.println(footswitch_state);
    // LED blinks every 0.3 seconds
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(100);              // wait for a second
  }
  else {
    Serial.print("In else case = ");
    Serial.println(footswitch_state);
    // LED blinks every 3 seconds
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(3000);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(3300);              // wait for a second
  }

    
}
  
  
  
 
 
 
 
 
 
 
 
/* 
void loop()
{
  footswitch_detect = digitalRead(FOOTSWITCH);
  if (footswitch_detect == HIGH) {
     Serial.print("BEGIN ");
    Serial.println(footswitch_state);
     footswitch_state++;
  }
  
  if (footswitch_state == 0) {
    Serial.print("In case 0 = ");
    Serial.println(footswitch_state);
    // LED blinks every 1 second
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);              // wait for a second
  }
  // press once
  else {
    Serial.print("In else case = ");
    Serial.println(footswitch_state);
    // LED blinks every 0.3 seconds
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(100);              // wait for a second
      footswitch_state = 0;
  }
} 
*/
 
 
/*
void loop()
{
  footswitch_detect = digitalRead(FOOTSWITCH);
  if (footswitch_detect == HIGH) {
     footswitch_state = (footswitch_state + 1) % 3; // have a total of three states
  }
  
  switch (footswitch_state) {
    // first mode that we enter, without pressing any botton
    case 0 :
      // LED blinks every 1 second
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);              // wait for a second
      break;
      
    // press once
    case 1 :
      // LED blinks every 0.3 seconds
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);              // wait for a second
      digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
      delay(100);              // wait for a second
      break;
    
    // press once more
    case 2 :
      // LED continue to be lit
      digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(5000);
      break;
      
    default:
      // testing only: turn light off
      digitalWrite(LED, LOW); 
      delay(5000); 

  }
}
  */
  /*
  // EFFECT: Distortion
  //Read the ADCs
  while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// wait for ADC 0, 1, 8, 9, 10 conversion complete.
  in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
  in_ADC1=ADC->ADC_CDR[6];               // read data from ADC1  
  POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
  POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
  POT2=ADC->ADC_CDR[12];                 // read data from ADC10     
 
  upper_threshold=map(POT0,0,4095,4095,2047);
  lower_threshold=map(POT0,0,4095,0000,2047);
  
  if(in_ADC0>=upper_threshold) in_ADC0=upper_threshold;
  else if(in_ADC0<lower_threshold)  in_ADC0=lower_threshold;
 
  if(in_ADC1>=upper_threshold) in_ADC1=upper_threshold;
  else if(in_ADC1<lower_threshold)  in_ADC1=lower_threshold;
 
  //adjust the volume with POT2
  out_DAC0=map(in_ADC0,0,4095,1,POT2);
  out_DAC1=map(in_ADC1,0,4095,1,POT2);
 
  //Write the DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);          //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);   //write on DAC
  dacc_set_channel_selection(DACC_INTERFACE, 1);          //select DAC channel 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);   //write on DAC
}
*/
