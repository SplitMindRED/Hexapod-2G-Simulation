#include "SML.h"

Hexapod::Hexapod(ros::NodeHandle np)
{
  nh = np;

  start_time_ = ros::Time::now();

  init();
}

void Hexapod::init()
{
  cmd_vel = nh.subscribe("cmd_vel", 100, &Hexapod::cmdVelCallback, this);
  servo_driver_topic = nh.advertise<hexapod_msgs::servo_driver_msg>("servo_driver_topic", 100);
}

void Hexapod::cmdVelCallback(geometry_msgs::Twist msg)
{
  velocity_ = msg;
  is_vel_updated_ = true;
}

void Hexapod::setServoAngle(uint8_t servo_num, double angle)
{
  //create msg of topic type
  hexapod_msgs::servo_driver_msg msg;
  msg.servo_number = servo_num;
  msg.angle = angle;
  servo_driver_topic.publish(msg);
}

//void Hexapod::moveLeg(uint8_t leg_num, double x, double y, double z)
//{
//  double* pointer = nullptr;
//  //get 3 angles
//  pointer = findAngles(leg_num, x, y, z);

//  //put them to leg structure
//  Leg[leg_num].q0 = *pointer;
//  Leg[leg_num].q1 = *(pointer + 1);
//  Leg[leg_num].q2 = *(pointer + 2);

//  //send commands to servos
//  setServoAngle(leg_num * 3, Leg[leg_num].q0);
//  setServoAngle(leg_num * 3 + 1, Leg[leg_num].q1);
//  setServoAngle(leg_num * 3 + 2, Leg[leg_num].q2);

//  //update current position of leg
//  Leg[leg_num].current_position.x = x;
//  Leg[leg_num].current_position.y = y;
//  Leg[leg_num].current_position.z = z;
//}

//unsigned long time_from_start = 0;
//unsigned long current_interruption_time = 0;
//uint16_t delta_interruption_time = 0;
//uint8_t channel_counter = 0;
//bool start_package = false;
//float channel[6];
//float Vx = 0, Vy = 0, Vz = 0;
//float input_roll = 0, input_pitch = 0, input_yaw = 0;
////float current_roll = 0, current_pitch = 0, current_yaw = 0;

//uint16_t delay_count = 0;

//bool servo_enable = false;

//int mpu = 0;
//uint8_t mpu_arr[14];

////geometry variables--------------------------
//const uint8_t OA = 37;
//const uint8_t AB = 44;
//const uint8_t BC = 83;
//double pi = 3.14;
//double q0rad, q1rad, q2rad, Qrad, Q0rad;
//double q0, q1, q2;
////--------------------------------------------

////movements and trajectory variables

//int16_t local_start_point[6][3] =
//{
//	 {X_OFFSET,       -Y_OFFSET,  -STARTHEIGHT},
//	 {X_OFFSET+30,    0,          -STARTHEIGHT},
//	 {X_OFFSET,       Y_OFFSET,   -STARTHEIGHT},
//	 {-X_OFFSET,      Y_OFFSET,   -STARTHEIGHT},
//	 {-X_OFFSET-30,   0,          -STARTHEIGHT},
//	 {-X_OFFSET,      -Y_OFFSET,  -STARTHEIGHT},
//};

//bool phase[2] = { 0, 1 };

//float diameter = DIAMETER;                   //60 mm aplitude in step

//float k = 0;
//float dH = DELTAHEIGHT;
//float H = STARTHEIGHT;

//unsigned long next_time = 1000;

//struct Legs Leg[6];
////--------------------------------------------


////hard delay, empty cycle
//void delay(int millisec)
//{
//	unsigned long start_time = time_from_start;
//	while (time_from_start <= (start_time + (millisec * 1000)))
//	{
//		//waiting
//	}
//}


//bool phaseControl(uint8_t group_num)
//{
//	float X, Y, Z;
//	float x, y, r;

//	X = Leg[group_num + 2].current_x;
//	Y = Leg[group_num + 2].current_y;
//	Z = Leg[group_num + 2].current_z;

//	//parameters in circle formula (x+x0)^2 + (y+y0)^2 = r^2
//	x = X - X_OFFSET + (2 * group_num * X_OFFSET);	//short form. if group = 0 -> -X_OFFSET, if group = 1 -> +X_OFFSET
//	y = Y - Y_OFFSET;
//	r = diameter / 2;

//	if (x*x + y*y >= r*r)
//	{
//		phase[group_num] = !phase[group_num];
//	}

//	return phase[group_num];
//}

//void findAngles(uint8_t leg_num, double x, double y, double z)
//{
//	double p = sqrt( x*x + y*y );
//	double OC = sqrt( p*p + z*z );
//	double AC = sqrt( (p-OA)*(p-OA) + z*z );

//	Qrad = atan(z / (p-OA));
//	Q0rad = acos((AB * AB + AC * AC - BC * BC) / (2 * AB * AC));

//	if (x == 0)
//	{
//		q0rad = 0;
//	}
//	else
//	{
//		//right side
//		if (leg_num <= 2)
//		{
//			q0rad = atan2(y, x);
//		}
//		//left side
//		else
//		{
//			if (x < 0)
//			{
//				q0rad = atan(y / x) + pi;
//			}
//			else
//			{
//				if (leg_num == 5)
//				{
//					q0rad = -atan(y / x) + pi; //FIX THIS SHIT!
//				}
//				else
//				{
//					q0rad = atan(y / x);
//				}
//			}
//		}
//	}

//	switch (leg_num)
//	{
//	case 0:
//		q0 = q0rad * 180 / pi + 135;		//back right
//		break;

//	case 1:
//		q0 = q0rad * 180 / pi + 90;		//mid right
//		break;

//	case 2:
//		q0 = q0rad * 180 / pi + 45;		//front right
//		break;

//	case 3:
//		q0 = q0rad * 180 / pi - 45;		//front left
//		break;

//	case 4:
//		q0 = q0rad * 180 / pi - 90;		//mid left
//		break;

//	case 5:
//		q0 = q0rad * 180 / pi - 135;		//back left
//		break;
//	}

//	//q0 = q0rad * 180 / pi + 45;		//front right
//	//q0 = q0rad * 180 / pi + 90;		//mid right
//	//q0 = q0rad * 180 / pi + 135;	//back right
//	//q0 = q0rad * 180 / pi - 45;		//front left
//	//q0 = q0rad * 180 / pi - 90;		//mid left
//	//q0 = q0rad * 180 / pi - 135;	//back left

//	q1rad = Qrad + Q0rad;
//	q1 = q1rad * 180 / pi + 90;

//	q2rad = acos( (AB*AB + BC*BC - AC*AC) / (2 * AB * BC) );
//	q2 = q2rad * 180 / pi;

//	Leg[leg_num].q0 = q0;
//	Leg[leg_num].q1 = q1;
//	Leg[leg_num].q2 = q2;
//}


