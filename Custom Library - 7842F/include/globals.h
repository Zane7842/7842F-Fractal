#pragma once
#include "main.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

namespace Globals {

extern pros::Controller master;
extern pros::MotorGroup left_mg;   // Creates a motor group with forwards ports 1 & 3 and reversed port 2
extern pros::MotorGroup right_mg;  // Creates a motor group with forwards port 5 and reversed ports 4 & 6   

extern pros::Imu imu_sensor;

extern pros::Rotation Forward_rotation;
extern pros::Rotation Sideways_rotation;


}