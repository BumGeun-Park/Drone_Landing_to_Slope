#include <iostream>
#include "ros/ros.h"
#include "rplidar_data/xyz.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#define x1 20
#define y1 20
#define z_bias 0.1

double dist1;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/Ground1_z",1);
	sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const rplidar_data::xyz& xyz)
	{
		int count = xyz.count;
		for(int i = 0; i<count; i++)
		  {
		    double dist = sqrt(pow((x1-xyz.x[i]),2)+pow((y1-xyz.y[i]),2));
		    if(dist<dist1)
		      {
			z.alpha = xyz.z[i];
			dist1 = dist;
			ROS_INFO("[x,y,z] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
		      }
		  }
		pub_.publish(z);
		dist1 = dist1+z_bias;
	}


private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
	rplidar_data::xyz xyz;
	rplidar_data::alpha z;
};

int main(int argc, char **argv)
{
    dist1 = 100.0;
    printf("\n");
    printf("\n");
    printf("Minimum distance data can be chosen in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("*****find_min1 node*****\n""\x1b[0m");
    printf("111111111111111111111111\n");
    ros::init(argc, argv, "find_min1");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}

