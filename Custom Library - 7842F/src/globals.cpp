#include "main.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "globals.h"


namespace Globals {

pros:: Imu imu_sensor(13);

pros::Rotation Forward_rotation(12);
pros::Rotation SideWays_rotation(1);
}