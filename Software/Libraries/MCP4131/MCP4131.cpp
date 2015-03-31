#include "MCP4131.h"


MCP4131::MCP4131(int csPin)
{
	//Sets SS/CS pin
	_cs = csPin;
	pinMode(_cs, OUTPUT);
	disable();

	//Increase the frequency when external pull ups are used
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV128);
	//SPI setup
}

void MCP4131::enable() {
	// take the SS pin low to select the chip:
	digitalWrite(_cs, LOW);
}

void MCP4131::disable() {
	// take the SS pin high to de-select the chip:
	digitalWrite(_cs, HIGH);
}

boolean MCP4131::increment() {
	enable();
	
	//  send in the address and value via SPI:
	byte ret1 =  0x02 & SPI.transfer(0x06); 
	//Needs bit 1 for error checking, Bit 2 inc

	disable();
	return (ret1 == 0);
}

boolean MCP4131::decrement() {
	enable();
	
	//  send in the address and value via SPI:
	byte ret1 = 0x02 & SPI.transfer(0x0A);
	//Needs bit 1 for error checking, Bit 3 Dec

	
	disable();
	return (ret1 == 0);
}

boolean MCP4131::readTCON() {
	enable();
	
	//  send in the address and value via SPI:
	byte ret1 = 0x02 & SPI.transfer(0x4F); 
	//At memory address 4 we read
	
	Tcon_Reg = SPI.transfer(0xFF); //Pull UP
	
	
	disable();
	return (ret1 == 0); //error checking
}

boolean MCP4131::initTCON() { 
	//Turns on Wiper 0, connects the terminals to the resistor network.
	enable();
	
	//  send in the address and value via SPI:
	byte ret1 = 0x02 & SPI.transfer(0x43); //Address of TCON
	byte ret2 = SPI.transfer(0x0F); //Set R0* no shutdown *connected 
	
	disable();
	readTCON();
	return (ret1 == 0); //error checking True if there is some.
}

boolean MCP4131::readStatus() {
	enable();
	
	//  send in the address and value via SPI:
	byte ret1 = 0x02 & SPI.transfer(0x5F); //Read Status
 
	Status_Reg = SPI.transfer(0xFF);//pull up
	
	disable();
	return (ret1 == 0);
}

boolean MCP4131::setTap(int value) {
	enable();

	//  send in the address and value via SPI:
	byte h = 0x03 & (value >> 8);
	byte l = 0x00FF & value;
	
	//Serial.print("HIGH: ");
	//Serial.println(h, BIN);
	//Serial.print("LOW: ");
	//Serial.println(l, BIN);
	
	h = h | 0x02; //make sure the error checking bit is high	

	byte ret1 = 0x02 & SPI.transfer(h); //we only want the error bit
	byte ret2 = SPI.transfer(l);

	disable();	
	//return (ret1 << 8) | ret2;
	readTap();
	return (ret1 == 0);
}

boolean MCP4131::readTap()
{
	enable();
	byte ret1 = 0x02 & SPI.transfer(0x0F); //Read Wiper 0, check error
 
	Wiper_Reg = SPI.transfer(0xFF); //pull up, value stored in wiper register
	
	disable();

	return (ret1 == 0);
}

