#include "ros/ros.h" 
#include "rplidar_data/alpha.h"

#define Kp 3 //Proportional gain

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/Kp_error4",1);
	sub_ = n_.subscribe("/error4",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const rplidar_data::alpha& error)
	{
	  rplidar_data::alpha output;
	  output.alpha = Kp*error.alpha;
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
    printf("This is proportional controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****proportional_controller4*****\n""\x1b[0m");
    ros::init(argc, argv, "proportional_controller4");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
