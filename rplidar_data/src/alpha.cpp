#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#define DEG2RAD(x) ((x)*M_PI/180.)
#define Angle_T 3

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/alpha",1);
	sub_ = n_.subscribe("/IMU_data",1,&SubscribeAndPublish::callback,this);
	}
	
        void callback(const sensor_msgs::Imu& IMU)
	{
            ROS_INFO("%f",IMU.orientation.x);
		rplidar_data::alpha output;
                double denominator = DEG2RAD(Angle_T)+0.05;
		output.alpha = 0.05/denominator;
		pub_.publish(output);
	}


private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
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
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
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
