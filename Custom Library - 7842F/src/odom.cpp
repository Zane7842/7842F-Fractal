#include "main.h"
#include "odom.h"
#include "globals.h"

namespace odom {

using namespace Globals;

int d = 2;

float to_rad (float degree) {
    return degree*(M_PI / 180.0);
}

    

void Odometry (float X_position, float Y_position, float prev_orientation_deg) {


float ForwardTracker_center_distance = 3.5;
float SidewaysTracker_center_distance = 0.5;
float orientation_offset = prev_orientation_deg;

//Previous values
float prev_ForwardTracker_position = 0;
float prev_SideWaysTracker_position = 0; 
//float X_position;
//float Y_position; 
//float prev_orientation_deg; 

//new values
float ForwardTracker_position = Forward_rotation.get_position();
float SideWaysTracker_position = SideWays_rotation.get_position(); 
float orientation_deg = imu_sensor.get_heading()+orientation_offset;

while (true) {

  float Forward_delta = ForwardTracker_position-prev_ForwardTracker_position;
  float SideWays_delta = SideWaysTracker_position-prev_SideWaysTracker_position;
  float Forward_delta_distance = (M_PI*d)*(Forward_delta / 360);
  float SideWays_delta_distance = (M_PI*d)*(SideWays_delta / 360);
  prev_ForwardTracker_position=ForwardTracker_position;
  prev_SideWaysTracker_position=SideWaysTracker_position;
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(prev_orientation_deg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  prev_orientation_deg=orientation_deg;

  float local_X_position;
  float local_Y_position;

  if (orientation_delta_rad == 0) {
    local_X_position = SideWays_delta_distance;
    local_Y_position = Forward_delta_distance;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((SideWays_delta_distance/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta_distance/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  float local_polar_angle;
  float local_polar_length;

  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2);

  float X_position_delta = local_polar_length*cos(global_polar_angle); 
  float Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;

        }

pros::lcd::print(1,"X_position %f", X_position);
pros::lcd::print(2,"Y_position %f", Y_position);
pros::lcd::print(3,"orientation_deg %f", orientation_deg);
    }

}



