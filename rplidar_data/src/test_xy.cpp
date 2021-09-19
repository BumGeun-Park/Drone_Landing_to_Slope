#include "ros/ros.h"  //ROS 기본 헤더파일
#include "rplidar_data/xyz.h"
#define DEG2RAD(x) ((x)*M_PI/180)
#define slope DEG2RAD(20) // 20 [deg]

int main(int argc, char **argv) //노드 메인 함수
{
    ros::init(argc,argv,"topic_publisher_test");
    ros::NodeHandle nh;
    ros::Publisher ros_test_pub=nh.advertise<rplidar_data::xyz>("/xyz",1);
    rplidar_data::xyz output;
    output.x.resize(81);
    output.y.resize(81);
    output.z.resize(81);
    output.count = 81;

    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        for(int i = 0;i<81;++i)
        {
            for(int j = 0;j<81;++j)
            {
                output.x[j] = 40-i;
                output.y[j] = -40+j;
                output.z[j] = 200 - 80*tan(slope)*((double)i/80);
                ROS_INFO("[x,y,z] = %f,%f,%f",output.x[j],output.y[j],output.z[j]);
            }
            ros_test_pub.publish(output);
        }
        loop_rate.sleep();
    }
    return 0;
}
