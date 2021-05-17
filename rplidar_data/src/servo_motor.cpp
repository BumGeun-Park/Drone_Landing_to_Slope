#include "ros/ros.h"
#include "rplidar_data/phi.h"
#include "math.h"
#include "sensor_msgs/JointState.h"

#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)

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
	    output.phi = input.position[0]; // ID1(라이다 모터)
	    pub_.publish(output);
	  }

private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
};


int main(int argc, char **argv) //노드 메인 함수
{
    printf("\n");
    printf("\n");
    printf("Dynamixel servo motor node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /scan\n""\x1b[0m");//바꾸자
    printf("\n");
    printf("\x1b[34m""Output : /polar\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****servo_motor node*****\n""\x1b[0m");
    ros::init(argc,argv,"servo_motor"); //노드명 초기화하는 함수
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
