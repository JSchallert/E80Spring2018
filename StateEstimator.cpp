#include "StateEstimator.h"
#include <math.h>
#include "Printer.h"
extern Printer printer;

inline float angleDiff(float a) {
  while (a<-PI) a += 2*PI;
  while (a> PI) a -= 2*PI;
  return a;
}

StateEstimator::StateEstimator(void)
  : DataSource("x,y,heading","float,float,float") // from DataSource
{}

void StateEstimator::init(double lat, double lon) {
  orig_lat = lat;
  orig_lon = lon;
 	state.x = 0;
  state.y = 0;
  state.heading = 0;
  cosOrigLat = cos(orig_lat*PI/180.0);
}

void StateEstimator::updateState(sensors_vec_t * imu_state_p, gps_state_t * gps_state_p) {
  // get x and y
  state.x = (gps_state_p->lon-orig_lon)*PI/180.0*RADIUS_OF_EARTH_M*cosOrigLat;
  state.y = (gps_state_p->lat-orig_lat)*PI/180.0*RADIUS_OF_EARTH_M;

  // get heading
  float heading_rad = imu_state_p->heading*PI/180.0; // convert to radians
  //heading_rad = heading_rad + 13.0/2.0/PI // add 13 degrees of magnetic declination to heading
  heading_rad = -heading_rad + PI/2.0; // adjust from 0=North, CW=(+) to 0=East, CCW=(+)
  state.heading = angleDiff(heading_rad);

}

String StateEstimator::printState(void) {
  int decimals = 2;
  String currentState = "State: x: " + String(state.x,decimals)
    + " y: " + String(state.y,decimals)
    + " h: " + String(state.heading,decimals);
  return currentState; 
}

size_t StateEstimator::writeDataBytes(unsigned char * buffer, size_t idx) {
  float * data_slot = (float *) &buffer[idx];
  data_slot[0] = state.x;
  data_slot[1] = state.y;
  data_slot[2] = state.heading;
  return idx + 3*sizeof(float);
}
