#pragma once

#include "api.h"
#include "pros/rotation.hpp"

// Your motors, sensors, etc. should go here.  Below are examples

 //inline pros::Motor intake(14);
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::Rotation vertical_rotation(12);
inline pros::Rotation horizantal_rotation(1);
