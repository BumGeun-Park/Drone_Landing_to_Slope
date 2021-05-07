#include "ros/ros.h" 
#include "rplidar_data/alpha.h"

double Avg;
double Dist;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/error3",1);
	sub_1 = n_.subscribe("/dist3",1,&SubscribeAndPublish::callback1,this);
	sub_2 = n_.subscribe("/avg_z",1,&SubscribeAndPublish::callback2,this);
	}
	
	void callback1(const rplidar_data::alpha& dist3)
	{
	  Dist = dist3.alpha;
	  feedback.alpha = Dist - Avg;
	  pub_.publish(feedback);	
	}

	void callback2(const rplidar_data::alpha& avg)
	{
	  Avg = avg.alpha;
	  feedback.alpha = Dist - Avg;
	  pub_.publish(feedback);	
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_1;
	ros::Subscriber sub_2;
	rplidar_data::alpha feedback;
};


int main(int argc, char **argv)
{
    Avg = 100.0;
    Dist = 100.0;
    printf("\n");
    printf("\n");
    printf("Feedback error can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****feedback3 node*****\n""\x1b[0m");
    ros::init(argc, argv, "feedback3");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}
