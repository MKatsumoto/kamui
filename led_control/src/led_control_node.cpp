#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "sh2interface/sh2_tx.h"
#include <iostream>
#include <cmath>

#define DUTY_STEP	0.01

std::string state = "Waiting";
bool send_flag = true;

void stateCallback(const std_msgs::String::ConstPtr& state_msg)
{
	state = state_msg -> data;
	send_flag = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "led_control_node");

	ros::NodeHandle n;
	sh2interface::sh2_tx txdata;


	// subscriber and publisher
  ros::Subscriber sub = n.subscribe("state", 1000, stateCallback);
	ros::Publisher pub = n.advertise<sh2interface::sh2_tx>("sh2_tx", 1000);
	
	ros::Rate loop_rate(100);	// ループ頻度を設定(100 Hz)

	txdata.command = 0xb0;	// command
	txdata.id = 0x21;		// id

  struct Duty
  {
    double red;
    double green;
    double blue;
  };

  Duty duty;
  duty.red = 0;
  duty.green = M_PI/3.0;
  duty.blue = M_PI*2.0/3.0;

	while (ros::ok())
  {
		if(state == "Normal" || state == "Following")	// Normal
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Tracking")	// Tracking
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xae;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Rotating")	// Rotating
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xae;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Closing")		// Closing
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Scanning")	// Scanning
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "GetInfo")	// GetInfo
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0x66;		// interval
		}
		else if(state == "Leaving")	// Leaving
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}
		else if(state == "Stop_temp")	// Stop_temp
		{	
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0x66;	// interval
		}
		else if(state == "Waiting")	// Waiting
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0xfe;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0x66;	// interval
		}
		else if(state == "Moving")	// Moving
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0xfe;	// blue
			txdata.data[3] = 0;	// interval
		}
		else if(state == "Teleope")	// Teleope
		{
			txdata.data[0] = 0xfe;	// red
			txdata.data[1] = 0x4c;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0x33;	// interval
		}
		else if(state == "Finish")	// Finish
		{
      txdata.data[0] = (uint8_t)(0xfe * sin(duty.red) * sin(duty.red));	// red
      txdata.data[1] = (uint8_t)(0xfe * sin(duty.green) * sin(duty.green));	// green
      txdata.data[2] = (uint8_t)(0xfe * sin(duty.blue) * sin(duty.blue));	// blue
			txdata.data[3] = 0;	// interval

      duty.red += DUTY_STEP;	if(duty.red > 2*M_PI)	duty.red = 0;
      duty.green += DUTY_STEP;	if(duty.green > 2*M_PI)	duty.green = 0;
      duty.blue += DUTY_STEP;	if(duty.blue > 2*M_PI)	duty.blue = 0;
		}
		else
		{
			txdata.data[0] = 0x00;	// red
			txdata.data[1] = 0x00;	// green
			txdata.data[2] = 0x00;	// blue
			txdata.data[3] = 0;		// interval
		}

		if(send_flag || state == "Finish")
		{
			pub.publish(txdata);
			send_flag = false;
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

  //TODO: 終了時に消灯 <-- sh2interfaceで実装済み？
	
	return 0;
}
