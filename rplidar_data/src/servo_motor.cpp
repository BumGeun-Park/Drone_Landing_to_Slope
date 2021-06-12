#include "ros/ros.h"
#include "rplidar_data/phi.h"
#include "math.h"
#include "sensor_msgs/JointState.h"

class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
        {
        pub_ = n_.advertise<rplidar_data::phi>("/phi",1);
        sub_ = n_.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback,this);
        }

	void callback(const sensor_msgs::JointState& input)
	  {
	    rplidar_data::phi output;
            output.phi = input.position[0]; // ID1(Lidar motor)
            output.vel = input.velocity[0]; // ID1(Lidar motor)
            output.sec = input.header.stamp.sec;
            output.nsec = input.header.stamp.nsec;

            //check///////////////////////////////////////////////////////////////////////////////
            printf("\x1b[34m""[checking operation]""\x1b[0m");
            ROS_INFO("Current phi: %f",output.phi);

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
    printf("Dynamixel servo motor node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /dynamixel_workbench/joint_states\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /phi\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****servo_motor node*****\n""\x1b[0m");
    ros::init(argc,argv,"servo_motor");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
