#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "rplidar_data/xyz.h"
#include "math.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>

#define DEG2RAD(x) ((x)*M_PI/180)
#define detect_angle 30 // (deg)
#define xb 0
#define zb 0
#define bias 0.01 //bias is 1cm
double phi = 0.0;
double Theta = DEG2RAD(detect_angle);
double motor_time;
double vel = 0.0;

using namespace message_filters;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_ = n_.advertise<rplidar_data::xyz>("/xyz",1);
        sub_1 = n_.subscribe("/scan",1,&SubscribeAndPublish::callback1,this);
        sub_2 = n_.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback2,this);
        /*message_filters::Subscriber<sensor_msgs::LaserScan> sub_1(n_, "/scan", 1);
        message_filters::Subscriber<sensor_msgs::JointState> sub_2(n_, "/dynamixel_workbench/joint_states", 1);
        TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::JointState> sync(sub_1, sub_2);
        boost::bind(&SubscribeAndPublish::callback1, _1, _2);
        sync.registerCallback(boost::bind(&SubscribeAndPublish::callback1,this, _1, _2));*/
    }

    void callback1(const sensor_msgs::LaserScan& input1)
    {
        int count = input1.scan_time / input1.time_increment;
        int new_count = 0;
        for(int i = 0; i < count; ++i)
        {
            double rad = input1.angle_min + input1.angle_increment * i;
            if (input1.ranges[i]<40.0 && input1.ranges[i]>0)
            {
                if(rad < DEG2RAD(-180)+Theta || rad > DEG2RAD(180)-Theta)
                {
                    new_count++;
                }
            }
        }

        double radian[new_count];
        double radius[new_count];
        int k = 0;

        for(int i = 0; i < count; ++i)
        {
            double rad = input1.angle_min + input1.angle_increment * i;
            if (input1.ranges[i]<40.0 && input1.ranges[i]>0)
            {
                if(rad < DEG2RAD(-180)+Theta || rad > DEG2RAD(180)-Theta)
                {
                    radian[k] = rad;
                    radius[k] = input1.ranges[i] + bias;
                    ++k;
                }
            }
        }
        int sec = input1.header.stamp.sec;
        double nsec = input1.header.stamp.nsec/1000000000.0;
        double Lidar_time = sec+nsec;
        double dt = Lidar_time - motor_time;
        dt = 0;
        phi = phi + vel*dt;

        rplidar_data::xyz xyz;
        xyz.x.resize(new_count);
        xyz.y.resize(new_count);
        xyz.z.resize(new_count);
        xyz.count = new_count;

        for(int i = 0; i<new_count ; ++i)
        {
            xyz.x[i] = -100*radius[i]*cos(radian[i])*sin(phi) + xb*cos(phi) + zb*sin(phi);
            xyz.y[i] = +100*radius[i]*sin(radian[i]);
            xyz.z[i] = -100*radius[i]*cos(radian[i])*cos(phi) + zb*cos(phi) - xb*sin(phi);
        }
        pub_.publish(xyz);
    }

    void callback2(const sensor_msgs::JointState& input2)
    {
        phi = input2.position[0];
        int sec = input2.header.stamp.sec;
        double nsec = input2.header.stamp.nsec/1000000000.0;
        vel = input2.velocity[0];
        motor_time = sec+nsec;
    }


private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    ros::Subscriber sub_3;

};

int main(int argc, char **argv)
{
    printf("\n");
    printf("\n");
    printf("Sperical coordinate can be transformed to cartesian coordinate in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input1 : /LaserScan\n""\x1b[0m");
    printf("\n");
    printf("\x1b[31m""Input2 : /JointState\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /xyz\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****assemble node*****\n""\x1b[0m");
    ros::init(argc, argv, "assemble");
    SubscribeAndPublish NH;
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
