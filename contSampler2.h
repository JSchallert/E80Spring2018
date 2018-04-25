#ifndef __CONTSAMPLER2_H__
#define __CONTSAMPLER2_H__

//number of samples to take
#define NUM_SAMPS 50
//time to take a single sample
#define SAMP_TIME 0.1

#include <Arduino.h>
#include "Pinouts.h"
#include "DataSource.h"
#include "TimingOffsets.h"

class contSampler : public DataSource 
{
public:
	contSampler(void);
	void init(void);
	
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
