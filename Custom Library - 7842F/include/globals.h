#pragma once
#include "main.h"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

namespace Globals {

extern pros::Imu imu_sensor;

extern pros::Rotation Forward_rotation;
extern pros::Rotation SideWays_rotation;
}