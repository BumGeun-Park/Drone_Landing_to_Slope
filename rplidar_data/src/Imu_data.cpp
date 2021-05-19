#include "ros/ros.h" 

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_ = n_.advertise<rplidar_data::packet>("/feedback",1);
        sub_ = n_.subscribe("/error",1,&SubscribeAndPublish::callback1,this);
	}
	
        void callback1(const rplidar_data::packet& error)
	{
         
	}

        void callback2(const sensor_msgs::JointState& input)
        {
           
        }
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
        ros::Subscriber sub_2;
};

int main(int argc, char **argv)
{
    Sum1 = 0.0;
    Sum2 = 0.0;
    Sum3 = 0.0;
    Sum4 = 0.0;
    printf("\n");
    printf("\n");
    printf("This is PID_controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****PID_controller*****\n""\x1b[0m");
    ros::init(argc, argv, "PID_controller");
    SubscribeAndPublish NH;
    t.time = ros::Time::now();
    int A = t.time.sec;
    double B = t.time.nsec/1000000000.0;
    ROS_INFO("Time checking: %f",A+B);
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
