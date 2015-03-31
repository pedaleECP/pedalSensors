/*
Orginal Version developed by declanshanaghy

  MCP4131.h - Library for Controlling MCP4131 Digital Pot.
  This one uses standard SPI library that is bundled with IDE
  Re-Created by Dennis Liang, November 19, 2011
  Released into the public domain.
/


*/


#ifndef MCP4131_h
#define MCP4131_h

#include "WProgram.h"
#include <SPI.h>

#define MOSI		11
#define MISO		12
#define SCK		13

#define MCP4131_MIN 0 //Tap value Min
#define MCP4131_MAX 128  //Tap value max



class MCP4131
{
public:
    MCP4131(int csPin);
	boolean initTCON();
	boolean readTCON();
	boolean readStatus();
	boolean  increment();
	boolean  decrement();
	boolean  setTap(int value);
	boolean readTap(); 
	byte Tcon_Reg; //TCON register
	byte Status_Reg; //Status Register
	byte Wiper_Reg; //Wiper Register
	
	
private:
    int _cs;
	void enable();
	void disable();
};

#endif

