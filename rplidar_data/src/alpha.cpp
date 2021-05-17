#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#include <cmath>

#define DEG2RAD(x) ((x)*M_PI/180.)
#define Angle_T 3

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_Roll = n_.advertise<rplidar_data::alpha>("/alpha_Roll",1);
        pub_Pitch = n_.advertise<rplidar_data::alpha>("/alpha_Pitch",1);
        pub_R = n_.advertise<rplidar_data::alpha>("/Roll",1);
        pub_P = n_.advertise<rplidar_data::alpha>("/Pitch",1);
	sub_ = n_.subscribe("/IMU_data",1,&SubscribeAndPublish::callback,this);
	}
	
        void callback(const sensor_msgs::Imu& IMU)
	{
            ROS_INFO("%f",IMU.orientation.x);
            double q0 = IMU.orientation.w;
            double q1 = IMU.orientation.x;
            double q2 = IMU.orientation.y;
            double q3 = IMU.orientation.z;
            double Roll = atan(2*(q0*q1+q2*q3)/(1-2*(q1*q1+q2*q2)));
            double Pitch = asin(2*(q0*q2-q3*q1));
            rplidar_data::alpha output_Roll;
            rplidar_data::alpha output_Pitch;
            rplidar_data::alpha output_R;
            rplidar_data::alpha output_P;
            double denominator_Roll = DEG2RAD(Angle_T)+abs(Roll);
            double denominator_Pitch = DEG2RAD(Angle_T)+abs(Pitch);
            double Roll_control = abs(Roll)/denominator_Roll;
            double Pitch_control = abs(Pitch)/denominator_Pitch;
            output_Roll.alpha = Roll_control;
            output_Pitch.alpha = Pitch_control;
            output_R.alpha = Roll_control;
            output_P.alpha = Roll_control;
            pub_Roll.publish(output_Roll);
            pub_Pitch.publish(output_Pitch);
            pub_R.publish(output_R);
            pub_P.publish(output_P);
	}

private:
	ros::NodeHandle n_;
        ros::Publisher pub_Roll;
        ros::Publisher pub_Pitch;
        ros::Publisher pub_R;
        ros::Publisher pub_P;
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
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}
