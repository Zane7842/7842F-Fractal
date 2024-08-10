#include "main.h"
#include "odom.hpp"
#include "globals.h"

namespace odom {
using namespace Globals; //Allows access rotation sensors and IMU_sensor

int d = 2; // Diameter of the tracking wheels (2")

/* Function to convert degrees to radiens */
float to_rad (float degree) {
    return degree*(M_PI / 180.0);
}

/*
 *Odometry Function: Does the odometry math to update position
 * Uses the Pilons arc method outline here: https://wiki.purduesigbots.com/software/

 *Parameters = (X position, Y Position, Orientation) -> of the Robot's starting position for an autonomous routine
 *Cordinates and heading are feild centred
   - Cartesian Cordinate:(0,0) = located in the center of the field
   - Orientation (heading) = clockwise-positive
   - Y axis is along double line on High Stakes Field
   - Posistive y direction is towards negative corners

 *Uses 2 Tracking Wheels
 *Tracking Wheels are equiped with their respective rotation sensors:
   - (ForwardTracker) uses the (Forward_rotation) sensor
   - (SideWaysTracker) uses the (SideWays_rotation) sensor
 */


void Odometry (float X_position = 0, float Y_position = 0, float prev_orientation_deg = 0) {

Forward_rotation.reset_position();
SideWays_rotation.reset_position();

float ForwardTracker_center_distance = 3.5; // Right-left distance from the tracking center to the left tracking wheel
float SidewaysTracker_center_distance = 0.5; // Forward-backward distance from the tracking center to the back tracking wheel
float orientation_offset = prev_orientation_deg; // Orientation_offset = how much the robots heading is offset from 0

/*Stores the current (soon to be a "previous value") encoder values in local variables*/
float prev_ForwardTracker_position = 0;
float prev_SideWaysTracker_position = 0; 

float ForwardTracker_position = Forward_rotation.get_position(); // Stores new encoder values via the rotation sensor's .get_position function
float SideWaysTracker_position = SideWays_rotation.get_position(); // Stores new orientation via the imu_sensor's .get_heading function
float orientation_deg = imu_sensor.get_heading()+orientation_offset; // Offsets the heading so it is field centred


while (true) {
  float Forward_delta = ForwardTracker_position-prev_ForwardTracker_position; // Finds the delta (change) of the ForwardTracker_position, by subtracting the previous value from its new value
  float SideWays_delta = SideWaysTracker_position-prev_SideWaysTracker_position; // Finds the delta (change) of the SideWaysTracker_position, by subtracting the previous value from its new value
  float Forward_delta_distance = (M_PI*d)*(Forward_delta / 360); // Converts position (in degrees) to distance (in inches).
  float SideWays_delta_distance = (M_PI*d)*(SideWays_delta / 360); // Converts position (in degrees) to distance (in inches).
  
  float orientation_rad = to_rad(orientation_deg); // Converts the "new" orientation from degrees to radiens
  float prev_orientation_rad = to_rad(prev_orientation_deg); // Converts the "previous" orientationfrom degrees to radiens
  float orientation_delta_rad = orientation_rad-prev_orientation_rad; // // Finds the delta (change) of the orientation, by subtracting the previous value from its new value
  
  float local_X_position; // Creates a local X cordinate variable (to be used to find the local offset)
  float local_Y_position; // Creates a local Y cordinate variable (to be used to find the local offset)

/*
If the robot's change in orientation is equal to 0 (no change in orientation since last cycle):
   * Simplifies the calculation of the local offset
*/
  if (orientation_delta_rad == 0) {
    local_X_position = SideWays_delta_distance;
    local_Y_position = Forward_delta_distance;
  }

/*
Otherwise, calculates the local offset using the following equation: (LINK to image to be put here)
Scaler multiplication = Everything inside the vector [] is multiplied by EVERYTHING outise of the vector
The vector has a size of 1x2 (top is X, bottom is Y)
In the code below, the vector is split up between the two local cordinates
*/
  else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((SideWays_delta_distance/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta_distance/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  float local_polar_angle; 
  float local_polar_length;

/*
If both the local X and Y position are equal to zero: 
   * Simplifies the calculation of the local_polar angle and length
*/
  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  }

/*
Otherwise, calculates the following conversion:
local_polar_angle = tan-1(y / x)
local_polar_length = √(x^2 + y^2)

Note: the c++ term "atan2" acts as tan-1, along with pproviding corretion for negative input values.
See the following link for more information on cartesian -> polar conversion: https://www.mathsisfun.com/polar-cartesian-coordinates.html
*/
  else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2); 

  float X_position_delta = local_polar_length*cos(global_polar_angle);  
  float Y_position_delta = local_polar_length*sin(global_polar_angle);

  // Updates new absolute position
  X_position+=X_position_delta; 
  Y_position+=Y_position_delta; 

  //Updates the Previous sensor values
  prev_ForwardTracker_position=ForwardTracker_position; 
  prev_SideWaysTracker_position=SideWaysTracker_position;
  prev_orientation_deg=orientation_deg;

  //Prints Values to the brain screen
  pros::lcd::print(0, "X Val: %.3f", X_position);
  pros::lcd::print(1, "Y Val: %.3f", Y_position);
  pros::lcd::print(2, "imu heading val: %.3f", imu_sensor.get_heading());

  /* 10 millisecond delay */
  pros::Task::delay(10);

      }

    }

}

/* Prints X and Y cordinate & orientation to the brain's screen 
pros::lcd::print(1,"X_position %f", X_position);
pros::lcd::print(2,"Y_position %f", Y_position);
pros::lcd::print(3,"orientation_deg %f", orientation_deg);

*/