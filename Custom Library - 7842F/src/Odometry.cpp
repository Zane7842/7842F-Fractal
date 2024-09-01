#include "liblvgl/llemu.hpp"
#include "main.h"
#include "Odometry.h"
#include "globals.h"
#include <string>

using namespace Globals; //Allows access rotation sensors and IMU_sensor


//namespace Odometry {

/* Function to convert degrees to radiens */
float to_rad (float degree) {return degree*(M_PI / 180.0);}


void Odom::start(){
    //Lambda function to continiously call update_position in a separate task
    pros::Task task ([&](){
        while (true) {
            this->update_position();
            pros::delay(10); 
        }
    });
}

void Odom::set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance, int TrackerWheel_diameter){
  this->TrackerWheel_diameter = TrackerWheel_diameter;
  this->ForwardTracker_center_distance = ForwardTracker_center_distance;
  this->SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

void Odom::set_position(float X_position = 0, float Y_position = 0, float orientation_deg = 0, float ForwardTracker_position = 0, float SidewaysTracker_position = 0){
  this->ForwardTracker_position = ForwardTracker_position;
  this->SidewaysTracker_position = SidewaysTracker_position;
  this->X_position = X_position;
  this->Y_position = Y_position;
  this->orientation_deg = orientation_deg;
  this->orientation_offset = orientation_deg;
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

void Odom::update_position(){


float ForwardTracker_position = Forward_rotation.get_position(); // Stores new encoder values via the rotation sensor's .get_position function
float SidewaysTracker_position = Sideways_rotation.get_position(); // Stores new orientation via the imu_sensor's .get_heading function
float orientation_deg = imu_sensor.get_heading()+orientation_offset;

//Debugging
  pros::lcd::print(3,"ForWardTracker_position:%.2f", ForwardTracker_position);
  pros::lcd::print(4,"SidewaysTracker_position:%.2f", SidewaysTracker_position);
  pros::lcd::print(5,"orientation_deg:%.2f", orientation_deg);

   // this-> always refers to the old version of the variable, so subtracting this->x from x gives delta x.
  float Forward_delta = ForwardTracker_position-this->ForwardTracker_position;
  float Sideways_delta = SidewaysTracker_position-this->SidewaysTracker_position;
  float Forward_delta_distance = (M_PI*TrackerWheel_diameter)*(Forward_delta / 360); // Converts position (in degrees) to distance (in inches).
  float Sideways_delta_distance = (M_PI*TrackerWheel_diameter)*(Sideways_delta / 360); // Converts position (in degrees) to distance (in inches).


//Debugging
  pros::lcd::print(6,"Forward_delta_distance:%.2f", Forward_delta_distance);
  pros::lcd::print(7,"Sideways_delta_distance:%.2f", Sideways_delta_distance);

  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(this->orientation_deg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  
  // //Debugging
  // pros::lcd::print(3,"orientation_delta_rad:%.2f", orientation_delta_rad);
  
  float local_X_position;
  float local_Y_position;

  if (orientation_delta_rad == 0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((Sideways_delta/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  // //Debugging
  // pros::lcd::print(4,"local_X_position:%.2f", local_X_position);
  // pros::lcd::print(5,"local_Y_position:%.2f", local_Y_position);

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

  // //Debugging
  // pros::lcd::print(6,"X_position_delta:%.2f", X_position_delta);
  // pros::lcd::print(7,"Y_position_delta:%.2f", Y_position_delta);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;


  //Updates the previous sensor values
  this->ForwardTracker_position=ForwardTracker_position;
  this->SidewaysTracker_position=SidewaysTracker_position;
  this->orientation_deg=orientation_deg;

   //Prints Values to the brain screen
  pros::lcd::set_text(0, "X Val: " + std::to_string(X_position));
  pros::lcd::set_text(1, "Y Val: " + std::to_string(Y_position));
  pros::lcd::set_text(2, "imu heading val: " + std::to_string(imu_sensor.get_heading()));
    }

//} odometry namespace

Odom odom;

