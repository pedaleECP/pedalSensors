#include <SPI.h>
#include <mcp4xxx.h>

using namespace icecave::arduino;

MCP4XXX* pot1;
MCP4XXX* pot2;

float a;

void setup()
{
    // Construct an instance of the MCP4XXX to manage the digipot.
    // The first parameter is the pin number to use for 'chip select' (CS), if you are
    // using the default SPI CS pin for your Arduino you can simply omit this parameter.
    pot1 = new MCP4XXX(10);
        pot2 = new MCP4XXX(9);

    Serial.begin(57600);
     Serial.println("-----------------------------------");
       pot1->set(0);
       pot2->set (127);

}

void loop()
{
    // Move the wiper to the lowest value

    // Move the wiper to the highest value
   // pot->set(pot->max_value());

    // Increment the wiper position by one
    pot1->increment();
    delay (1000);
       int sensor1Value = analogRead(A0);
          float voltage1 = sensor1Value * (5.0 / 1023.0);
          
       int sensor2Value = analogRead(A1);
          float voltage2 = sensor2Value * (5.0 / 1023.0); 
                  Serial.print("POT 1: ");

        Serial.print(pot1->get());
        Serial.print(" | ");
                          Serial.print("VOLT 1: ");

           Serial.print(voltage1);
        Serial.print(" | ");

            Serial.print("POT 2: ");

        Serial.print(pot2->get());
        Serial.print(" | ");
                          Serial.print("VOLT 2: ");
                                     Serial.println(voltage2);

       // Serial.println(pot1->get());


    // Decrement the wiper position by one
   pot2->decrement();
   if (pot1->get() > 127)
   {
           pot1->set(0);
}
  if (pot2->get() < 1)
   {
           pot2->set(127);
}
}
