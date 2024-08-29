#pragma once
#include "main.h"
#include "globals.h"



class Odom
{
public:
  void start();
private:
  int TrackerWheel_diameter;
  float ForwardTracker_center_distance;
  float SidewaysTracker_center_distance;
  float ForwardTracker_position;
  float SidewaysTracker_position;
public:
  float X_position;
  float Y_position;
  float orientation_deg;
  float orientation_offset;
  void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
  void update_position(); //task?
  void set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance, int TrackerWheel_diameter);
};
extern Odom odom;
