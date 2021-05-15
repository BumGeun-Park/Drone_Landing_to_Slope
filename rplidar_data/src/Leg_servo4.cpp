#include "ros/ros.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#include "sensor_msgs/JointState.h"
#define Leg_length 35
#define Arm_length 15
#define motor_z 0
#define Leg4 4
#define start_deg 10
#define DEG2RAD(x) ((x)*M_PI/180)
#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)

class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
        {
        pub_ = n_.advertise<rplidar_data::alpha>("/Leg4_z",1);
        sub_ = n_.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback,this);
        }

	void callback(const sensor_msgs::JointState& input)
	  {
	    rplidar_data::alpha output;
	    output.alpha = Leg_length + Arm_length*tan(DEG2RAD(start_deg)+input.position[Leg4]); // ID5(다리 모터)
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
    ros::init(argc,argv,"Leg_servo4");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
