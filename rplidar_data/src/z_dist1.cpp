#include "ros/ros.h" 
#include "rplidar_data/alpha.h"

double Ground1_z;
double Leg1_z;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/dist1",1);
	sub_1 = n_.subscribe("/Ground1_z",1,&SubscribeAndPublish::callback1,this);
	sub_2 = n_.subscribe("/Leg1_z",1,&SubscribeAndPublish::callback2,this);
	}
	
	void callback1(const rplidar_data::alpha& Ground)
	{
	  Ground1_z = Ground.alpha;
	  z.alpha = Ground1_z - Leg1_z;
	  pub_.publish(z);	
	}

	void callback2(const rplidar_data::alpha& Leg)
	{
	  Leg1_z = Leg.alpha;
	  z.alpha = Ground1_z - Leg1_z;
	  pub_.publish(z);	
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_1;
	ros::Subscriber sub_2;
	rplidar_data::alpha z;
};


int main(int argc, char **argv)
{
    Ground1_z = 100.0;
    Leg1_z = 0.0;
    printf("\n");
    printf("\n");
    printf("Distance to ground can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****z_dist1 node*****\n""\x1b[0m");
    ros::init(argc, argv, "z_dist1");
    SubscribeAndPublish NH;
    ros::Rate loop_rate(8000);
    ros::spin();
 
    return 0;
}
