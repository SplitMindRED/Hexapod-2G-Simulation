#include "servo_driver.h"

ServoDriver::ServoDriver(ros::NodeHandle n)
{
  nh = n;

  init();
}

void ServoDriver::init()
{
  initSubscribers();
  initPublishers();

  for(uint8_t i = 0; i < 18; i++)
  {
    servo_angle[i].data = 0;
  }

}

void ServoDriver::initPublishers()
{
  //publishers to all servo topics
  servo[0] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_0_joint_position_controller/command", 100);
  servo[1] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_1_joint_position_controller/command", 100);
  servo[2] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_2_joint_position_controller/command", 100);
  servo[3] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_3_joint_position_controller/command", 100);
  servo[4] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_4_joint_position_controller/command", 100);
  servo[5] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_5_joint_position_controller/command", 100);
  servo[6] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_6_joint_position_controller/command", 100);
  servo[7] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_7_joint_position_controller/command", 100);
  servo[8] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_8_joint_position_controller/command", 100);
  servo[9] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_9_joint_position_controller/command", 100);
  servo[10] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_10_joint_position_controller/command", 100);
  servo[11] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_11_joint_position_controller/command", 100);
  servo[12] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_12_joint_position_controller/command", 100);
  servo[13] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_13_joint_position_controller/command", 100);
  servo[14] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_14_joint_position_controller/command", 100);
  servo[15] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_15_joint_position_controller/command", 100);
  servo[16] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_16_joint_position_controller/command", 100);
  servo[17] = nh.advertise<std_msgs::Float64>("/hexapod_model/servo_17_joint_position_controller/command", 100);
}

void ServoDriver::initSubscribers()
{
  servo_driver_topic = nh.subscribe("servo_driver_topic", 100, &ServoDriver::servoCommandCallBack, this);
}

void ServoDriver::servoCommandCallBack(hexapod_msgs::servo_driver_msg msg)
{
  //get new angle for specific servo and put it to array
  servo_angle[msg.servo_number].data = msg.angle;
  int a = msg.servo_number;
  std::cout << "Servo: " << a << " new angle: " << msg.angle << std::endl;
}

void ServoDriver::moveServo()
{
  //move all servo
  for(uint8_t i = 0; i < 18; i++)
  {
    servo[i].publish(servo_angle[i]);
  }
}
