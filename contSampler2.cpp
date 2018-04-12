
#include "contSampler2.h"
#include <math.h>
//#include <stdio.h>
#include "Printer.h"
extern Printer printer;
//String message;

contSampler::contSampler(void)
    : DataSource("Velocity","float")
{}

void contSampler::updateSample(void)
{  
  for (int i=0; i<NUM_SAMPS; i++){
    samps[i] = analogRead(ENCODER_PIN);
    //will always spin in the same direction
  }
}

String contSampler::printSample(void)
{
  String printString = "ENCODER:";
  for (int i=0; i<NUM_SAMPS; i++) {
    printString += " ";
    printString += String(samps[i]);
  }
  return printString; //printer.printValue(0, printString);
}

void contSampler::computeVelocity(void){
    //0V = 0 = 0 deg, 5V=1024 = 360 degrees
    //2.84 /degree
    sumdeltas =0;
    for (int i=0; i<NUM_SAMPS-1; i++){
        deltas[i]=(samps[i+1])-(samps[i]);
        if(deltas[i] < 0){
            //case where we go from 5V to 0V (360 to 0)
            // delta would be 0-1024 = -1024
            deltas[i] +=1024;
        }
        //convert to degrees
        deltas[i] = deltas[i]/2.84;
        sumdeltas += deltas[i];
    }
    velocity = sumdeltas/(NUM_SAMPS*SAMP_TIME);

}

size_t contSampler::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = velocity;
  return idx + sizeof(float);
}

