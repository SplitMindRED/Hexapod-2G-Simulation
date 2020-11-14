#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include "math.h"
#include "stdint.h"
#include "hexapod_msgs/servo_driver_msg.h"

//Hexapod parametres
//offset from (0, 0) on XY plane in every local coordinate system of leg
#define X_OFFSET                    50
#define Y_OFFSET                    50

#define STARTHEIGHT                 70

//height of step
#define DELTAHEIGHT                 30

//diameter of step circle. distance of step
#define DIAMETER                    60

#define LOOP_RATE                   1000


class Hexapod
{
public:
  Hexapod(ros::NodeHandle np);

  //inverse kinematics for leg
  //get 3 servo angles from X, Y, Z coordinates
  double* findAngles(uint8_t leg_num, double x, double y, double z);

  //move specific leg to exact point in local coodrinate system of this leg
  void moveLeg(uint8_t leg_num, double x, double y, double z);

  //move specific servo to exact angle
  void setServoAngle(uint8_t servo_num, double angle);

  hexapod_msgs::servo_driver_msg a;

private:
  /* * * * * * * * * * * * * * * FUNCTIONS * * * * * * * * * * * * * */
  void init();

  void cmdVelCallback(geometry_msgs::Twist msg);

  /* * * * * * * * * * * * * * * VARIABLES * * * * * * * * * * * * * */
  //ros node handle for using in Hexapod class
  ros::NodeHandle nh;

  //publisher to servo driver
  ros::Publisher servo_driver_topic;
  //subscriber to topic with velocity commands
  ros::Subscriber cmd_vel;

  //variable for new velocity that we get from cmd_vel topic
  geometry_msgs::Twist velocity_;

  //flag which says if velocity is updated, works alongside with velocity_ variable
  bool is_vel_updated_;

  //start ROS time
  ros::Time start_time_;
  //current ROS time
  ros::Time current_time_;
  //time stamp for some action
  ros::Time last_time_;

  //array of structures for each Leg
  //contains all information of legs
  struct Legs
  {
     //f(t) coordinates functions
     float Xt, Yt, Zt;

     //current local position of leg (x, y, z)
     geometry_msgs::Vector3 current_position;

     //new local position of leg (x, y, z)
     geometry_msgs::Vector3 target_position;

     //angles of each servo of leg
     double q0, q1, q2;

  } Leg[6];

};

//extern unsigned long time_from_start;
//extern unsigned long current_interruption_time;
//extern uint16_t delta_interruption_time;
//extern uint8_t channel_counter;
//extern bool start_package;
//extern float channel[6];
//extern float Vx, Vy, Vz;
//extern float input_roll, input_pitch, input_yaw;
////extern float current_roll, current_pitch, current_yaw;

//extern uint16_t delay_count;

//extern bool servo_enable;

////servo angles--------------------------
//extern double q0, q1, q2;
////--------------------------------------

////movements and trajectory variables

//extern int16_t local_start_point[6][3];
//extern bool phase[2];

//extern float diameter;

//extern float k;
//extern float dH;
//extern float H;

//extern int mpu;
//extern uint8_t mpu_arr[14];

//extern unsigned long next_time;

////all cordinates of this structure are
////in local coordinate system of each leg
//extern struct Legs
//{
//   //f(t) coordinates functions
//   float Xt, Yt, Zt;

//   //current local position of leg (x, y, z)
//   float current_x;
//   float current_y;
//   float current_z;

//   //new local position of leg (x, y, z)
//   float target_x;
//   float target_y;
//   float target_z;

//   double q0, q1, q2;

//} Leg[6];




////HEXAPOD MOVEMENTS---------------------------------------------------------------------------
//void findAngles(uint8_t leg_num, double x, double y, double z);

//void moveLeg(uint8_t leg_num, double x, double y, double z);
////END OF HEXAPOD MOVEMENTS--------------------------------------------------------------------
