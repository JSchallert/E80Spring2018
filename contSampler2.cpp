 
#include "contSampler2.h"
#include <math.h>
//#include <stdio.h>
#include "Printer.h"
extern Printer printer;

contSampler::contSampler(void)
    : DataSource("Velocity","float")
{}

void contSampler::init(void){
  pinMode(ENCODER_PIN, INPUT);

  printer.printMessage("Initialized Encoder at " + String(millis()),10);
}

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
        if(deltas[i] < -50 ){
            //case where we go from 5V to 0V (360 to 0)
            // delta would be 0-1024 = -1024 go from 50 to 917
            deltas[i] +=917;
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
  /*for (int i = 0; i < NUM_SAMPS; i++) {
    data_slot[i] = samps[i];
  }
  */
  return idx + sizeof(float);
}

