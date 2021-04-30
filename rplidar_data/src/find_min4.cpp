#include <iostream>
#include "ros/ros.h"
#include "rplidar_data/xyz.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#define x4 20
#define y4 20
#define z_bias 0.1

double dist4;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/Ground4_z",1);
	sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const rplidar_data::xyz& xyz)
	{
		int count = xyz.count;
		for(int i = 0; i<count; i++)
		  {
		    double dist = sqrt(pow((x4-xyz.x[i]),2)+pow((y4-xyz.y[i]),2));
		    if(dist<dist4)
		      {
			z.alpha = xyz.z[i];
			dist4 = dist;
			ROS_INFO("[x,y,z] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
		      }
		  }
		pub_.publish(z);
		dist4 = dist4+z_bias;
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
    dist4 = 100.0;
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
    printf("*****find_min4 node*****\n""\x1b[0m");
    printf("444444444444444444444444\n");
    ros::init(argc, argv, "find_min4");
    SubscribeAndPublish NH;
    ros::spin();
 
    return 0;
}
