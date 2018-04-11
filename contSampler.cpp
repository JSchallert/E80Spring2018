
#include "contSampler.h"
#include <math.h>
#include <stdio.h>
#include "Printer.h"
extern Printer printer;
String message;

contSampler::contSampler(void){
    //Nothing to do?
}

void contSampler::updateSamples(void)
{  
  for (int i=0; i<NUM_SAMPS; i++){
    samps[i] = analogRead(ENCODER_PIN);
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

size_t contSampler::writeDataBytes(unsigned char * buffer, size_t idx)
{
  int * data_slot = (int *) &buffer[idx];
  for (int i=0; i<NUM_SAMPS; i++) {
    data_slot[i] = samps[i];
  }
  return idx + NUM_SAMPS*sizeof(int);
}

void contSampler::padding(int number, byte width, String & str) {
	int currentMax = 10;
	for (byte i = 1; i < width; i++) {
		if (number < currentMax) {
			str.concat("0");
		}
		currentMax *= 10;
	}
	str.concat(number);
}

void contSampler::init(void) {
	Serial.print("Initializing SD Card for contSampler... ");
  if (!SD.begin()) {
    Serial.println("failed!");
    return;
  }
  Serial.println("done!");

	unsigned int number = 0;
	String numstr = "";
	padding(number, 3, numstr);
	String finalname = SMP_FILENAME_BASE + numstr + ".bin";
	finalname.toCharArray(logfilename, SMP_FILENAME_BUFFERLEN);

	while(SD.exists(logfilename)) {
		number++;
		numstr = "";
		padding(number, 3, numstr);
		finalname = SMP_FILENAME_BASE + numstr + ".bin";
		finalname.toCharArray(logfilename, SMP_FILENAME_BUFFERLEN);
	}

	finalname = HEADINGS_FILENAME_BASE + numstr + ".txt";
	finalname.toCharArray(headingfilename, SMP_FILENAME_BUFFERLEN);

	message = "contSampler: Using log file name " + String(logfilename);
	printer.printMessage(message,30);

	String headingStr = "Analog Encoder";
	String dataTypeStr = "int";
	
	headingStr += "\n"+dataTypeStr;

	file = SD.open(headingfilename, FILE_WRITE);

	// if the file exists, use it:
  if (file) {
    file.println(headingStr);
    file.close();
	}

	printer.printMessage("Creating log file",10);
	file = SD.open(logfilename, FILE_WRITE);
	if(!file) {
		message = "Logger: error creating " + String(logfilename);
		printer.printMessage(message,0);
	} else {
		file.close();
	}

	// if exiting without error
	keepLogging = true;
}

bool contSampler::log(void){
	// record data from sources
	size_t idx = 0;
	unsigned char buffer[BYTES_PER_BLOCK];
	for(size_t i = 0; i < num_samples; ++i) {
		if (idx >= BYTES_PER_BLOCK) {
			printer.printMessage("Too much data per log. Increase BYTES_PER_BLOCK or reduce data", 2);
		}
	}

	// write data to SD
	if (writtenBlocks >= FILE_BLOCK_COUNT) {
		printer.printMessage("Current file size limit reached. Change FILE_BLOCK_COUNT to fix. Stopping logging for now.",0);
		keepLogging = false;
	}

	file = SD.open(logfilename, FILE_WRITE);
	if (file) {
		int bytes = file.write(&buffer[0],BYTES_PER_BLOCK);
		if (!bytes) {
			printer.printMessage("Logger: Error printing to SD",0);
		}
	}
	file.close();

	writtenBlocks++;
	keepLogging = true;
}

String contSampler::printState(void){
	String printString = "Logger: ";
	if(keepLogging) {
		printString += "Just logged buffer " + String(writtenBlocks) + " to SD.";
	} else {
		printString += "LOGGING ERROR, LOGGING HAS STOPPED";
	}

	return printString;
}
