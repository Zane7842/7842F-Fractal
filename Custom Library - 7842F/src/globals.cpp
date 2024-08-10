#include "main.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "globals.h"


namespace Globals {

pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6

pros:: Imu imu_sensor(13);

pros::Rotation Forward_rotation(12);
pros::Rotation SideWays_rotation(1);
}