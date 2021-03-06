#include <ros/ros.h>
#include "serial/serial.h"
#include <iostream>
#include <vector>
#include <string>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "stbee_serial/setPwm.h"
#include <math.h>

#define RX_DATA_SIZE 5
#define TIMEOUT 1000
#define CONTROL_F 40 // rate of sending control input [Hz]

#define PI 3.14159265358979

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

int flag_pub;
uint8_t tx_data;
int8_t duty;

bool set_pwm(stbee_serial::setPwm::Request  &req,
		 	 stbee_serial::setPwm::Response &res)
{
	if(req.pwm > 100)
    {
        duty = 100;
    }
    else if(req.pwm < -100)
    {
        duty = -100;
    }
    else
    {
        duty = req.pwm;
    }
    tx_data = (uint8_t)(duty + 0x7f);
    flag_pub = 1;
}

void pwmCallback(const std_msgs::Int8::ConstPtr& msg)
{
    if(msg->data > 100)
    {
        duty = 100;
    }
    else if(msg->data < -100)
    {
        duty = -100;
    }
    else
    {
        duty = msg->data;
    }
    tx_data = (uint8_t)(duty + 0x7f);
    flag_pub = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stbee_serial_node");
    ros::NodeHandle n;

    // Publisher and Subscriber
    ros::Publisher gimbang_pub = n.advertise<std_msgs::Float64>("gimbal_angle", 10); //[rad]
    ros::Subscriber pwm_sub = n.subscribe("gimbal_pwm", 1000, pwmCallback);
	ros::ServiceServer service = n.advertiseService("set_gimbal_pwm", set_pwm);

    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial("/dev/ttyACM0", 115200, serial::Timeout::simpleTimeout(1000));

    cout << "Is the serial port open?";
    if(my_serial.isOpen())
      cout << " Yes." << endl;
    else
      cout << " No." << endl;

    // motor stop
    tx_data = 0x7f;
    my_serial.write(&tx_data, 1);
    ros::Duration(0.2).sleep();

    short count;
    int sync;
    std_msgs::Float64 gimbal_angle;

    uint8_t rx_data[RX_DATA_SIZE];
    uint8_t buff;

    double angle_data[CONTROL_F];
    for(int i = 0; i < CONTROL_F; i++)
    {
        angle_data[i] = 0;
    }

    int offset = 0;
    if(ros::param::has("/stbee_serial_node/modify_delay"))
    {
        int offset_;
        ros::param::get("/stbee_serial_node/modify_delay", offset_);
        if(offset_ >= 0 && offset_ < CONTROL_F)
        {
            offset = offset_;
        }
    }
	double offset_angle_manual = 0.0;
	if(ros::param::has("/stbee_serial_node/modify_angle"))
    {
        ros::param::get("/stbee_serial_node/modify_angle", offset_angle_manual);
    }

    flag_pub = 0;
	double offset_angle = 0.0;

    double t0, t;
    t0 = ros::Time::now().toSec();

    ros::Rate r(10);
    while(ros::ok())
    {
        if(flag_pub == 1)
        {
            //tx_data = 0x0a;
            int result = my_serial.write(&tx_data, 1);
            //ros::Duration(0.2).sleep();
            flag_pub = 0;
            ROS_INFO("Send duty = %d %s", duty, (result==1)?"success!":"fail...");//debug
        }

        for(int i=0; i<RX_DATA_SIZE; i++)
        {
            my_serial.read(&rx_data[i], 1);
        }

        if(rx_data[0] == 0xff && rx_data[1] == 0xff)
        {
            count = (short)(rx_data[2] << 8) + (short)(rx_data[3]);

            sync = (int)rx_data[4];

            //cout << count << "  " << sync;
            //cout << endl;

            t = ros::Time::now().toSec();
            if(t - t0 > 1.0/CONTROL_F)
            {
                for(int i = CONTROL_F-1; i > 0; i--)
                {
                    angle_data[i] = angle_data[i-1];
                }
				
				if(duty > 0)
				{
					offset_angle = -0.138006024;//-atanf(2.5/18.0);
				}
				else if(duty < 0)
				{
					offset_angle = 0.138006024;//atanf(2.5/18.0);
				}
				else
				{
					offset_angle = 0;
				}
				
                // gimbal angle
                angle_data[0] = -(count - 0x7fff) / (128.0*32.0) * 2*PI + PI;
                gimbal_angle.data = angle_data[offset] + offset_angle + offset_angle_manual;

               //ROS_INFO("%f  %f", t, t0);//debug
               //ROS_INFO("%d %d", count, sync);//debug
               gimbang_pub.publish(gimbal_angle);
               t0 = t;
            }
        }
        else
        {
            //cout << "Fault." << endl;//debug
            my_serial.read(&buff, 1);
        }

        ros::spinOnce();
        //r.sleep();
    }

	// motor stop
    tx_data = 0x7f;
    my_serial.write(&tx_data, 1);
    ros::Duration(0.2).sleep();
}
