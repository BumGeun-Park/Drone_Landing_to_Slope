#include "ros/ros.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#define Leg_length 35
#define Arm_length 15

int main(int argc, char **argv) //노드 메인 함수
{
	ros::init(argc,argv,"Leg_servo4"); //노드명 초기화하는 함수
	ros::NodeHandle nh; //ROS 시스템과 통신을 위한 노드 핸들 선언


	ros::Publisher ros_test_pub=nh.advertise<rplidar_data::alpha>("/Leg4_z",1);

	rplidar_data::alpha z;
	double t = 0.0;	
	while(ros::ok())
	{
		t++;
		double p = t*(3.14/2000);
		double theta = 1.5*sin(p);
		z.alpha = Leg_length-15*tan(theta);
		ros_test_pub.publish(z); //메시지 발행
	}
	return 0;
}
