// Licensed under a Creative Commons Attribution 3.0 Unported License.
// Based on rcarduino.blogspot.com previous work.
// www.electrosmash.com/pedalshield
 
/*octaver.ino creates an octave-up or octave-down signal from the original one.*/

//Inclusion de notre code.

      

 //Lignes  Originales
 
          int in_ADC0, in_ADC1;  //variables for 2 ADCs values (ADC0, ADC1)
          int POT0, POT1, POT2, out_DAC0, out_DAC1; //variables for 3 pots (ADC8, ADC9, ADC10)
          int LED = 3;
          int FOOTSWITCH = 7; 
          int TOGGLE = 2; 
           
          #define MAX_DELAY 10000
          uint16_t sDelayBuffer0[MAX_DELAY-1];
          uint16_t sDelayBuffer1[MAX_DELAY-1];
          unsigned int write_pt=0;
          unsigned int read_pt_A=0, read_pt_B= MAX_DELAY/2;
          unsigned int Delay_Depth, increment, divider=0, buffer0, buffer1;
          
 //Vaiables de MovingTones
 

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
  //Lignes de MovingTones
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
  
  
  
      //Original Code of Octaver
          //turn on the timer clock in the power management controller
            pmc_set_writeprotect(false);
            pmc_enable_periph_clk(ID_TC4);
           
            //we want wavesel 01 with RC 
            TC_Configure(TC1, 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
            TC_SetRC(TC1, 1, 238); // sets <> 44.1 Khz interrupt rate 109
            TC_Start(TC1, 1);
           
            // enable timer interrupts on the timer
            TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
            TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;
           
            //Enable the interrupt in the nested vector interrupt controller 
            //TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number 
            //(=(1*3)+1) for timer1 channel1 
            NVIC_EnableIRQ(TC4_IRQn);
           
            //ADC Configuration
            ADC->ADC_MR |= 0x80;   // DAC in free running mode.
            ADC->ADC_CR=2;         // Starts ADC conversion.
            ADC->ADC_CHER=0x1CC0;  // Enable ADC channels 0,1,8,9 and 10  
           
            //DAC Configuration
            analogWrite(DAC0,0);  // Enables DAC0
            analogWrite(DAC1,0);  // Enables DAC0
}
 
void loop()
{
 // updateScreen();
  // readSaveButton();
  
  
  //Original Lines
            //Read the ADCs
            while((ADC->ADC_ISR & 0x1CC0)!=0x1CC0);// wait for ADC 0,1,8,9,10 conversion complete.
            in_ADC0=ADC->ADC_CDR[7];               // read data from ADC0
            in_ADC1=ADC->ADC_CDR[6];               // read data from ADC1  
            //POT0=ADC->ADC_CDR[10];                 // read data from ADC8        
            //POT1=ADC->ADC_CDR[11];                 // read data from ADC9   
            //POT2=ADC->ADC_CDR[12];                 // read data from ADC10   
          
                readSensor();

  

  
    
}
 
void TC4_Handler() //Interrupt at 44.1KHz rate (every 22.6us)
{
  TC_GetStatus(TC1, 1); //Clear status interrupt to be fired again.
 
  //Store current readings  
  sDelayBuffer0[write_pt] = in_ADC0;
  sDelayBuffer1[write_pt] = in_ADC1;
 
  //Adjust Delay Depth based in pot2 position.
  Delay_Depth = MAX_DELAY-1;
 
  //Increse/reset delay counter.
  write_pt++;
  if(write_pt >= Delay_Depth) write_pt = 0; 
 
  out_DAC0 = ((sDelayBuffer0[read_pt_A]));
  out_DAC1 = ((sDelayBuffer1[read_pt_B]));
 
  if (p0>2700)
  { 
    read_pt_A = read_pt_A + 2;
    read_pt_B = read_pt_B + 2;
  }
 else if (p0>1350)
  {
    read_pt_A = read_pt_A + 1;
    read_pt_B = read_pt_B + 1;
  }
 else
 {
   divider++;
   if (divider>=2)
   {
      read_pt_A = read_pt_A + 1;
      read_pt_B = read_pt_B + 1;
      divider=0;
    }
  }
 
  if(read_pt_A >= Delay_Depth) read_pt_A = 0; 
  if(read_pt_B >= Delay_Depth) read_pt_B = 0; 
 
  //Add volume control with POT2
  out_DAC0=map(out_DAC0,0,4095,1,p2);
  out_DAC1=map(out_DAC1,0,4095,1,p2);
 
  //Write the DACs
  dacc_set_channel_selection(DACC_INTERFACE, 0);       //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC0);//write on DAC
  dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
  dacc_write_conversion_data(DACC_INTERFACE, out_DAC1);//write on DAC
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
    p0 = angles[0] * 4096 / 120;
    p1 = angles[1] * 4096 / 90;
    p2 = angles[2] * 4096 / 90;
    
    
    
  }
}


