#include "ros/ros.h"
#include "rplidar_data/packet.h"
#include "math.h"
#include "sensor_msgs/JointState.h"

// Hardware spec
#define Leg_length 33.8
#define Arm_length 10
#define motor_z -9
//

#define Leg1 1
#define Leg2 2
#define Leg3 3
#define Leg4 4
#define DEG2RAD(x) ((x)*M_PI/180)
#define RAD2DATA(x) ((x)*(2047/M_PI)+2048)
#define DATA2RAD(x) ((x)-2048)*(M_PI/2047)


class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
        {
        pub_ = n_.advertise<rplidar_data::packet>("/Leg_z",1);
        sub_ = n_.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback,this);
        }

	void callback(const sensor_msgs::JointState& input)
	  {
            rplidar_data::packet output;
            output.packet.resize(4);
            output.packet[0] = motor_z + Leg_length + Arm_length*tan(input.position[Leg1]); // ID2(다리 모터)
            output.packet[1] = motor_z + Leg_length + Arm_length*tan(input.position[Leg2]); // ID3(다리 모터)
            output.packet[2] = motor_z + Leg_length + Arm_length*tan(input.position[Leg3]); // ID4(다리 모터)
            output.packet[3] = motor_z + Leg_length + Arm_length*tan(input.position[Leg4]); // ID5(다리 모터)

            //check///////////////////////////////////////////////////////////////////////////////
            printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
            ROS_INFO("Leg Position: %f, %f, %f, %f",output.packet[0],output.packet[1],output.packet[2],output.packet[3]);
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
    printf("\x1b[31m""Input : /dynamixel_workbench/joint_states\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****Leg servo_motor node*****\n""\x1b[0m");
    ros::init(argc,argv,"Leg_servo1");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
