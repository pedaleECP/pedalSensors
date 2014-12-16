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


void setup() {

    // intialize our own Serial display on the computer
    Serial.begin(57600); 
  
    // initialize the XBee's serial port transmission
    Serial1.begin(57600);
    
    Serial.println("SCREEN READY TO PRINT");
    Serial1.println("XBEE SERIAL1 INITIATED");
    
    lastByte = (byte)0;
}

void loop() {
  
  if (Serial1.available()) {
    incomingByte = Serial1.read();
    Serial.print("byte1 = ");
    Serial.print(incomingByte, HEX);
    Serial.print("       ");
  }

    //Serial.print("lastByte = ");
    //Serial.print(lastByte, HEX);
    //Serial.print("   |    ");

    int1 = assembleInt(lastByte, incomingByte); 
    
    if (int1 == -21846) {
       
      Serial.println("HEADER IS IDENTIFIED");
      
      /*
       incomingByte = Serial1.read();
       byteArray[0] = incomingByte;
    
       incomingByte = Serial1.read();
       byteArray[1] = incomingByte;
        
       counter = assembleInt(byteArray[0], byteArray[1]);
       Serial.print("int afterwards = ");
       Serial.print(counter, DEC);
       Serial.println("       ");
       */
      
      readAngles();
    }
  
    lastByte = incomingByte;

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
  
  // print to computer screen
  Serial.print("currentInt = ");
  Serial.print(currentInt, DEC);
  Serial.print("   |    ");
  Serial.println("");
  
  return currentInt;
}


void readAngles() {
  for (int i = 0; i < 3; i++) {
     if (Serial1.available()) {
        incomingByte = Serial1.read();
        byteArray[0] = incomingByte;
        
        incomingByte = Serial1.read();
        byteArray[1] = incomingByte;
      }
      angles[i] = assembleInt(byteArray[0], byteArray[1]);
      
      Serial.print("Angle");
      Serial.print(i);
      Serial.print(angles[i], DEC);
      Serial.print("   |    ");
      Serial.println("");
  }  
}
