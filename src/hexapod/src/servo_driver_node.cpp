#include "hexapod/servo_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_driver");

  ros::NodeHandle n;

  ServoDriver ServoDriver(n);

  ros::Rate loop_rate(LOOP_FREQUENCY);

  while (ros::ok())
  {
    ros::spinOnce();

    ServoDriver.moveServo();
    loop_rate.sleep();
  }


  return 0;
}
