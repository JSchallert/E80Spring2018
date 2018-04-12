#ifndef __TIMINGOFFSETS_H__
#define __TIMINGOFFSETS_H__

// all times are recorded in units of ms
// the offsets define when tasks happen within a loop period
#define LOOP_PERIOD 100
#define PRINTER_LOOP_OFFSET 0
#define IMU_LOOP_OFFSET 10
#define GPS_LOOP_OFFSET 20
#define ADC_LOOP_OFFSET 30
#define STATE_ESTIMATOR_LOOP_OFFSET 40
#define P_CONTROL_LOOP_OFFSET 45
#define LED_LOOP_OFFSET 50
//change
#define SMP_LOOP_OFFSET 55
#define LOGGER_LOOP_OFFSET 75


#endif