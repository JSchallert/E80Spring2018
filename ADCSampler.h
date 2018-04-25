#include <Arduino.h>
#include "MotorDriver.h"
#include "DataSource.h"
#include "Pinouts.h"

/*
 * ADCSampler implements SD logging for the ADC channels
 */

#define NUM_PINS 15

class ADCSampler : public DataSource
{
public:
  ADCSampler(void);

  // Managing state
  int sample [NUM_PINS];
  void updateSample(void);
  String printSample(void);

  // Write out
  size_t writeDataBytes(unsigned char * buffer, size_t idx);

  int lastExecutionTime = -1;
};
