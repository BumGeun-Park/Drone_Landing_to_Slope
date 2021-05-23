#include "ros/ros.h" 
#include "sensor_msgs/LaserScan.h"
#include "rplidar_data/polar.h"
#define bias 0.01 //bias is 1cm

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::polar>("/polar",1);
	sub_ = n_.subscribe("/scan",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const sensor_msgs::LaserScan& input)
	{
	int count = input.scan_time / input.time_increment;
	rplidar_data::polar output;
	output.count = count;
	output.radian.resize(count);
	output.radius.resize(count);
                for(int i = 0; i < count; ++i)
		  {
			double radian = (input.angle_min + input.angle_increment * i);
			double radius = input.ranges[i];
                        output.radian[i] = radian;
                        output.radius[i] = radius + bias;

                        //check///////////////////////////////////////////////////////////////////////////////
                        printf("\x1b[34m""[checking operation]""\x1b[0m");
                        ROS_INFO("[r,theta] = [%f,%f]", output.radius[i], output.radian[i]);
		  }
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
    printf("Raw data can be transformed in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /scan\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /polar\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****Raw_data2Information node*****\n""\x1b[0m");
    ros::init(argc, argv, "Raw_data2Information");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
