#include "ros/ros.h"
#include "rplidar_data/phi.h"
#include "math.h"

int main(int argc, char **argv) //노드 메인 함수
{
	ros::init(argc,argv,"servo_motor"); //노드명 초기화하는 함수
	ros::NodeHandle nh; //ROS 시스템과 통신을 위한 노드 핸들 선언


	ros::Publisher ros_test_pub=nh.advertise<rplidar_data::phi>("/phi",1);

	rplidar_data::phi msg;
	double t = 0.0;	
	while(ros::ok())
	{
		t++;
		double p = t*(3.14/2000);
                msg.phi = 0.33*sin(p);
		ros_test_pub.publish(msg); //메시지 발행
	}
	return 0;
}
