#include "ros/ros.h"
#include "rplidar_data/alpha.h"

double z_arr[4] = {1200.0,1200.0,1200.0,1200.0};

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/avg_z",1);
	sub_1 = n_.subscribe("/dist1",1,&SubscribeAndPublish::callback1,this);
	sub_2 = n_.subscribe("/dist2",1,&SubscribeAndPublish::callback2,this);
	sub_3 = n_.subscribe("/dist3",1,&SubscribeAndPublish::callback3,this);
	sub_4 = n_.subscribe("/dist4",1,&SubscribeAndPublish::callback4,this);
	}
	
	void callback1(const rplidar_data::alpha& z)
	{
		z_arr[0] = z.alpha;
		z_avg.alpha = (z_arr[0] + z_arr[1] + z_arr[2] + z_arr[3])/4 ;
		pub_.publish(z_avg);

	}
	void callback2(const rplidar_data::alpha& z)
	{
		z_arr[1] = z.alpha;
		z_avg.alpha = (z_arr[0] + z_arr[1] + z_arr[2] + z_arr[3])/4 ;
		pub_.publish(z_avg);

	}
	void callback3(const rplidar_data::alpha& z)
	{
		z_arr[2] = z.alpha;
		z_avg.alpha = (z_arr[0] + z_arr[1] + z_arr[2] + z_arr[3])/4 ;
		pub_.publish(z_avg);

	}
	void callback4(const rplidar_data::alpha& z)
	{
		z_arr[3] = z.alpha;
		z_avg.alpha = (z_arr[0] + z_arr[1] + z_arr[2] + z_arr[3])/4 ;
		pub_.publish(z_avg);

	}


private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_1;
	ros::Subscriber sub_2;
	ros::Subscriber sub_3;
	ros::Subscriber sub_4;
	rplidar_data::alpha z_avg;
};

int main(int argc, char **argv)
{
    printf("\n");
    printf("\n");
    printf("Average height of end effector of leg can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input1 : /z_height1\n""\x1b[0m");
    printf("\x1b[31m""Input2 : /z_height2\n""\x1b[0m");
    printf("\x1b[31m""Input3 : /z_height3\n""\x1b[0m");
    printf("\x1b[31m""Input4 : /z_height4\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /avg_z\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****avg_z node*****\n""\x1b[0m");
    printf("Initial value : %f\n",(z_arr[0] + z_arr[1] + z_arr[2] + z_arr[3])/4);
    ros::init(argc, argv, "avg_z");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}
