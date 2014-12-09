byte incomingByte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  Serial1.begin(57600);

  Serial.println("ready to recieve");
    Serial1.println("give me");

}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.print(millis());
  //Serial1.println("SENDING FROM NANO");
  
  // if there is bytes available coming from the serial port
if (Serial1.available()) {
 // Serial.println("one message");

// set the values to the ‘incomingByte’ variable
incomingByte = Serial1.read();

// write the value to the pin 11
Serial.print(incomingByte);

}
Serial.println();


}
