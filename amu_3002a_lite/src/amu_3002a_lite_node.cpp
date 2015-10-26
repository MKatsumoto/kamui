#include <ros/ros.h>
#include "std_msgs/String.h"
#include "can_msgs/CANFrame.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>

// Coefficients for converting AMU data to physical values
static const double BIT_TO_DEG = 0.0078125;
static const double BIT_TO_DEG_PER_SEC = 0.015625;
static const double BIT_TO_G= 0.0001220703125;
// Convert unit
static const double DEG_TO_RAD = (3.14159265/180.0);
static const float  G_TO_MS2   = 9.80665;


class AMU{
public:
    AMU();
    void start();
    void stop();
    void receiveDataCallback(const can_msgs::CANFrame::ConstPtr &msg);
    double getAngle(unsigned char lsb, unsigned char msb);
    double getAngularVel(unsigned char lsb, unsigned char msb);
    double getAcc(unsigned char lsb, unsigned char msb);

private:
    ros::Publisher  can_pub_;
    ros::Publisher  imu_pub_;
    ros::Subscriber can_sub_;
};


/// --------------------------------------------------
/// Implementation
/// --------------------------------------------------
AMU::AMU()
{
    ros::NodeHandle node;
    can_pub_ = node.advertise<can_msgs::CANFrame>("/can_bus_tx",1);
    imu_pub_ = node.advertise<sensor_msgs::Imu>("/imu/data",1);
    can_sub_ = node.subscribe("/can_bus_rx", 3, &AMU::receiveDataCallback, this);
//    amu_service_ = node.advertiseService("amu_command",commandAMU);   // TODO: Use service to start/stop AMU
}


void AMU::receiveDataCallback(const can_msgs::CANFrame::ConstPtr& msg)
{
    // Restore angle
    double angle[3] = {0.0, 0.0, 0.0};  // [deg]
    if(msg->id == 0x41)
    {
        for(int i = 0; i < 3; i++)
        {
            angle[i] = getAngle(msg->data[i*2], msg->data[i*2+1]);
        }
        ROS_INFO("Roll: %3.1f, Pitch: %3.1f, Yaw: %3.1f [deg]", angle[0], angle[1], angle[2]);
    }

    // Restore angular velocity
    double angular_vel[3] = {0.0, 0.0, 0.0};  // [deg/sec]
    if(msg->id == 0x42)
    {
        for(int i = 0; i < 3; i++)
        {
            angular_vel[i] = getAngularVel(msg->data[i*2], msg->data[i*2+1]);
        }
        ROS_INFO("Roll vel: %3.1f, Pitch vel: %3.1f, Yaw vel: %3.1f [deg/sec]", angular_vel[0], angular_vel[1], angular_vel[2]);
    }

    // Restore translational acceleration
    double acc[3] = {0.0, 0.0, 0.0};  // [G]
    if(msg->id == 0x43)
    {
        for(int i = 0; i < 3; i++)
        {
            acc[i] = getAcc(msg->data[i*2], msg->data[i*2+1]);
        }
        ROS_INFO("Acc_x: %3.1f, Acc_y: %3.1f, Acc_z: %3.1f [m/sec^2]", acc[0], acc[1], acc[2]);
    }

    // Publish Topic
    sensor_msgs::Imu data;

    // Angle [rad] -> Quaternion
    // Remark: Coordinate between KAMUI and AMU is different. (y- and z-axis are reverse)
    //         ** KAMUI **
    //           x+: forward, y+: left, z+: above
    //         **  AMU  **
    //           x+: forward, y+: right, z+: below
    // You must reverse the sign of pitch and yaw, and also corresponding angular velocities.
    double roll = angle[0] * DEG_TO_RAD;
    double pitch = -angle[1] * DEG_TO_RAD;
    double yaw = -angle[2] * DEG_TO_RAD;
    data.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    // Angular velocity [rad/s]
    data.angular_velocity.x = angular_vel[0] * DEG_TO_RAD;
    data.angular_velocity.y = -angular_vel[1] * DEG_TO_RAD;
    data.angular_velocity.z = -angular_vel[2] * DEG_TO_RAD;

    // Linear acceleration [m/sec^2]
    data.linear_acceleration.x = acc[0] * G_TO_MS2;
    data.linear_acceleration.y = -acc[1] * G_TO_MS2;
    data.linear_acceleration.z = -acc[2] * G_TO_MS2;

    // Covariance
    // TODO: Check the definition of covariance / 20150807, Katsumoto
    data.orientation_covariance[0] = -1;
    data.angular_velocity_covariance[0] = -1;
    data.linear_acceleration_covariance[0] = -1;

    // Publish
    imu_pub_.publish(data);
}

double AMU::getAngle(unsigned char lsb, unsigned char msb)
{
    unsigned int angle_lsb = static_cast<unsigned int>(lsb);
    unsigned int angle_msb = static_cast<unsigned int>(msb);
    int angle_data = static_cast<int>(angle_lsb + (angle_msb << 8));
    double angle;
    if (angle_data < 32768) // 2^15 = 32768
    {
        return angle = static_cast<double>(angle_data) * BIT_TO_DEG;
    }
    else
    {
        return angle = -(static_cast<double>(65536 - angle_data) * BIT_TO_DEG);   // 2^16 = 65536
    }
}

double AMU::getAngularVel(unsigned char lsb, unsigned char msb)
{
    unsigned int angular_vel_lsb = static_cast<unsigned int>(lsb);
    unsigned int angular_vel_msb = static_cast<unsigned int>(msb);
    int angular_vel_data = static_cast<int>(angular_vel_lsb + (angular_vel_msb << 8));
    double angular_vel;
    if (angular_vel_data < 32768) // 2^15 = 32768
    {
        return angular_vel = static_cast<double>(angular_vel_data) * BIT_TO_DEG_PER_SEC;
    }
    else
    {
        return angular_vel = -(static_cast<double>(65536 - angular_vel_data) * BIT_TO_DEG_PER_SEC);   // 2^16 = 65536
    }
}

double AMU::getAcc(unsigned char lsb, unsigned char msb)
{
    unsigned int acc_lsb = static_cast<unsigned int>(lsb);
    unsigned int acc_msb = static_cast<unsigned int>(msb);
    int acc_data = static_cast<int>(acc_lsb + (acc_msb << 8));
    double acc;
    if (acc_data < 32768) // 2^15 = 32768
    {
        return acc = static_cast<double>(acc_data) * BIT_TO_G;
    }
    else
    {
        return acc = -(static_cast<double>(65536 - acc_data) * BIT_TO_G);   // 2^16 = 65536
    }
}

void AMU::start()
{
    can_msgs::CANFrame command;
    command.id = 0x50;
    command.data.resize(2);
    command.data[0] = 0x01;
    command.data[1] = 0x01;
    can_pub_.publish(command);
    ROS_INFO("Start AMU");
}

void AMU::stop()
{
    can_msgs::CANFrame command;
    command.id = 0x50;
    command.data.resize(2);
    command.data[0] = 0x00;
    command.data[1] = 0x00;
    can_pub_.publish(command);
    ROS_INFO("Stop AMU");
}

/// --------------------------------------------------
/// Main
/// --------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "amu_3002a_lite_node");
    AMU amu;
    ros::Duration(1.0).sleep();
    ros::Rate loop_rate(100);
    amu.start();
    loop_rate.sleep();
    amu.start();

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    amu.stop();

    return 0;
}
