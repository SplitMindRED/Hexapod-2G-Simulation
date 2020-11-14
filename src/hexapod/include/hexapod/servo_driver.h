#pragma once

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "hexapod_msgs/servo_driver_msg.h"
#include "math.h"
#include <iostream>
#include "stdint.h"

#define LOOP_FREQUENCY    100 //Hz

/* * * * This node publish servo angles at constant frequency * * * */

class ServoDriver
{
public:
  ServoDriver(ros::NodeHandle n);

  void moveServo();

private:
  /* * * * * * * * * * * * * * * FUNCTIONS * * * * * * * * * * * * * */

  void init();

  void initSubscribers();

  void initPublishers();

  void servoCommandCallBack(hexapod_msgs::servo_driver_msg msg);

  /* * * * * * * * * * * * * * * VARIABLES * * * * * * * * * * * * * */

  ros::NodeHandle nh;

  //get new angles from  hexapod_main_controller
  ros::Subscriber servo_driver_topic;
  //18 publishers to servo topics
  ros::Publisher servo[18];

  std_msgs::Float64 servo_angle[18];


};
