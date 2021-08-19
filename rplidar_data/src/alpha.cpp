#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "rplidar_data/packet.h"
#include "math.h"
#include <cmath>

#define DEG2RAD(x) ((x)*M_PI/180.)
#define Angle_T 3

#define length 22.3
#define max_angle 27
#define scaling (length*tan(DEG2RAD(max_angle))/2)/DEG2RAD(max_angle)

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_s_RP = n_.advertise<rplidar_data::packet>("/s_RP",1);
        pub_RP_alpha = n_.advertise<rplidar_data::packet>("/alpha_RP",1);
        sub_ = n_.subscribe("/mavros/imu/data",1,&SubscribeAndPublish::callback,this);
	}
	
        void callback(const sensor_msgs::Imu& IMU)
	{
            double q0 = IMU.orientation.w;
            double q1 = IMU.orientation.x;
            double q2 = IMU.orientation.y;
            double q3 = IMU.orientation.z;
            double Roll = atan(2*(q0*q1+q2*q3)/(1-2*(q1*q1+q2*q2)));
            double Pitch = -asin(2*(q0*q2-q3*q1));
            double Scaled_roll = Roll * scaling;
            double Scaled_pitch = Pitch * scaling;
            rplidar_data::packet RP;
            rplidar_data::packet RP_alpha;
            RP.packet.resize(2);
            RP_alpha.packet.resize(2);
            double denominator_Roll = DEG2RAD(Angle_T)+abs(Roll);
            double denominator_Pitch = DEG2RAD(Angle_T)+abs(Pitch);
            double Roll_alpha = abs(Roll)/denominator_Roll;
            double Pitch_alpha = abs(Pitch)/denominator_Pitch;
            RP.packet[0] = Roll_alpha*Scaled_roll;
            RP.packet[1] = Pitch_alpha*Scaled_pitch;
            RP_alpha.packet[0] = Roll_alpha;
            RP_alpha.packet[1] = Pitch_alpha;

            //check///////////////////////////////////////////////////////////////////////////////
            printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
            ROS_INFO("[Roll,pitch,R_alpha,P_alpha] = %f, %f, %f, %f",Roll,Pitch,Roll_alpha,Pitch_alpha);
            pub_s_RP.publish(RP);
            pub_RP_alpha.publish(RP_alpha);
	}

private:
	ros::NodeHandle n_;
        ros::Publisher pub_s_RP;
        ros::Publisher pub_RP_alpha;
	ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    printf("\n");
    printf("\n");
    printf("Weight can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /IMU_data\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output1 : /alpha_Roll\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output2 : /alpha_Pitch\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output3 : /Roll\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output4 : /Pitch\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****alpha*****\n""\x1b[0m");
    ros::init(argc, argv, "alpha");
    SubscribeAndPublish NH;
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
    ros::spinOnce();
    loop_rate.sleep();
    }
 
    return 0;
}
