#ifndef __PINOUTS_H__
#define __PINOUTS_H__

#define LOOP_LED 7
#define USER_SWITCH 2

#define MOTOR_L_FORWARD 3		// M4: pins 3,4
#define MOTOR_L_REVERSE 20
#define MOTOR_R_FORWARD 4		// M1: pins 6,9
#define MOTOR_R_REVERSE 21
#define MOTOR_V_FORWARD	5		// M3  pins 16, 17
#define MOTOR_V_REVERSE	22
#define MOTOR_X_FORWARD 6		// M2 pins 20,22
#define MOTOR_X_REVERSE 23

//pin on teensy encoder is inputing to A02 ==16
#define ENCODER_PIN 16

//IR LED
#define LED_PIN 9

/* TOF */
#define SPEAKER_PIN 20
#define MIC_PIN 23

#endif
