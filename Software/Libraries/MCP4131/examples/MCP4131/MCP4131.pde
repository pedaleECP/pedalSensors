/*
  MCP4131 example
 
Example to test MCP4131 from sparkfun: http://www.sparkfun.com/products/10613
 
 Circuit:
 * MCP4131 attached to pins 10, 11, 12, 13
 * SDO is connected to a 4.7K resitor before it connect to the shared SDI/SDO Pin
 * Analog inputs attached to pins A0 through A5 (optional)
 
Example Created by Dennis Liang on 11/20/2011
Orginal MCP4161 implementation from https://github.com/declanshanaghy/ArduinoLibraries/tree/master/MCP4161

 
 */

#include <SPI.h>
#include <MCP4131.h>

//setup an intance of MCP4131
MCP4131 MCP4131(10); // with a CS pin of 10
int incomingByte = 0;

long previousMillis = 0;        // will store last time LED was updated

int tapValue = 1;        // will store last time LED was updated


void setup(void)
{
    // start serial port
  Serial.begin(115200);
  Serial.println("MCP4231 Test:");
  
  
  MCP4131.setTap(MCP4131_MIN); //Sets the pot to 0.
  //MCP4131.setTap(MCP4131_MAX); //Sets the pot to MAX.
  if (MCP4131.initTCON())  //init the pot, connect wiper0, and turn ON pot
  {
    Serial.println("Init Error!");
  }
  
  
}

void loop(void)
{
  delay(1000);
  
  if (MCP4131.readTCON()==false)
  {
    Serial.print("TCON:");
    Serial.println(MCP4131.Tcon_Reg, BIN); ///wipper and shutdown control bits //should be all one
  }
    
  if (MCP4131.readStatus()==false)
  {
    Serial.print("STATUS:");
    Serial.println(MCP4131.Status_Reg, BIN); ///wipper and shutdown control bits //should be all one
  }
  
   if (MCP4131.readTap()==false)
  {
  Serial.print("WIPER:");
  Serial.println(MCP4131.Wiper_Reg, DEC); //Where the wiper position
  }
 

     
    if (MCP4131.setTap(tapValue))
    {
      Serial.println("setTap Error!");
    }
    else
    {
      tapValue++;    
      if (tapValue > 128)
      {
        tapValue = 0;
      }
      Serial.print("WIPER AFTER SET:");
      Serial.println(MCP4131.Wiper_Reg, DEC); //Wiper register is updated
    }
    
    if (MCP4131.increment())
    {
      Serial.println("increment Error!"); //Try to Increment
    }
    
     if (MCP4131.readTap()==false)
      {
        Serial.print("WIPER AFTER INCREMENT:");
        Serial.println(MCP4131.Wiper_Reg, DEC);
       }
     
    
    if (MCP4131.decrement())
    {
      Serial.println("decrement Error!"); //Try to decrement
    }
   
     if (MCP4131.readTap()==false)
      {
      Serial.print("WIPER AFTER DECREMENT:");
      Serial.println(MCP4131.Wiper_Reg, DEC);
      }
 
  
 
}
