#ifndef __CONTSAMPLER_H__
#define __CONTSAMPLER_H__
#include <stdio.H>

#define SMP_FILENAME_BASE "SMP"
#define SMP_FILENAME_BUFFERLEN 20
#define HEADINGS_FILENAME_BASE "inf"
// buffered logging
// number of 512B blocks in the log file
#define FILE_BLOCK_COUNT 8192 // should last over 10 min
#define BYTES_PER_BLOCK 256
// number of blocks in the buffer
#define BUFFER_BLOCK_COUNT 5
#define MAX_NUM_DATASOURCES 10

//number of samples to take
#define NUM_SAMPLES 50
//pin on teensy encoder is inputing to A01 ==15
#define ENCODER_PIN 15

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "Pinouts.h"

class contSampler {
public:
	contSampler(void);
	
	// Managing state
  	int samps [NUM_SAMPS];
  	void updateSample(void);
  	String printSample(void);

  	// Write out
  	size_t writeDataBytes(unsigned char * buffer, size_t idx);

	int lastExecutionTime = -1;

    // run after all dataSources have been registered
	void init(void);

	// records all data at the time it's called to the SD
	bool log(void);

	String printState(void);
	int lastExecutionTime = -1;
	bool keepLogging = false;

private:
	void padding(int number, byte width, String & str);

	char logfilename[SMP_FILENAME_BUFFERLEN];
	char headingfilename[SMP_FILENAME_BUFFERLEN];
	File file;

	uint32_t writtenBlocks = 0;
};
#endif
