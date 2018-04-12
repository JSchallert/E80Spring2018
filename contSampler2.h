#ifndef __CONTSAMPLER2_H__
#define __CONTSAMPLER2_H__

//number of samples to take
#define NUM_SAMPS 50
//pin on teensy encoder is inputing to A02 ==16
#define ENCODER_PIN 16
//time to take a single sample
#define SAMP_TIME 5

#include <Arduino.h>
#include "Pinouts.h"
#include "DataSource.h"
#include "TimingOffsets.h"

class contSampler : public DataSource 
{
public:
	contSampler(void);
	
	// Managing state
  	int samps [NUM_SAMPS];
  	void updateSample(void);
  	String printSample(void);
    float deltas[NUM_SAMPS-1];
	float sumdeltas;
    float velocity;
	void computeVelocity(void);

    //write out
    size_t writeDataBytes(unsigned char * buffer, size_t idx);
	int lastExecutionTime = -1;


};
#endif
