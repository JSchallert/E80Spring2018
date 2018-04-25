#include "ADCSampler.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

ADCSampler::ADCSampler(void) 
  : DataSource("A00,A01,EncoderA02,NewPressureA03,A10,A11,A12,A13,A14,A15,A16,A17,A18,PD1A19,PD2A20",
               "int,int,int,int,int,int,int,int,int,int,int,int,int,int,int") // from DataSource
{}


void ADCSampler::updateSample(void)
{
  // maps pins to variable names
  // A10-A13 are pins 34-37, A14 is pin 40, rest same as pinout picture
  // pins A12-A13 and A15-A20 are on surface mount pads underneath the Teensy
  // pins A10-A14 are _NOT_ 5V tolerarant!  Everyone else is.  (2x check)
  const int pinmap[NUM_PINS] =  {14,15,16,17,34,35,36,37,40,26,27,28,29,30,31}; 
  for (int i=0; i<NUM_PINS; i++){
    sample[i] = analogRead(pinmap[i]);
  }
}

String ADCSampler::printSample(void)
{
  String printString = "ADC:";
  for (int i=0; i<NUM_PINS; i++) {
    printString += " ";
    printString += String(sample[i]);
  }
  return printString; //printer.printValue(0, printString);
}

size_t ADCSampler::writeDataBytes(unsigned char * buffer, size_t idx)
{
  int * data_slot = (int *) &buffer[idx];
  for (int i=0; i<NUM_PINS; i++) {
    data_slot[i] = sample[i];
  }
  return idx + NUM_PINS*sizeof(int);
}
