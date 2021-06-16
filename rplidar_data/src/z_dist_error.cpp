#include "ros/ros.h"
#include "rplidar_data/packet.h"

double Ground_z[4];
double Leg_z[4];
double dist[4];
double avg;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_ = n_.advertise<rplidar_data::packet>("/error",1);
        sub_1 = n_.subscribe("/Ground_z",1,&SubscribeAndPublish::callback1,this);
        sub_2 = n_.subscribe("/Leg_z",1,&SubscribeAndPublish::callback2,this);
    }

    void callback1(const rplidar_data::packet& Ground)
    {
        error.packet.resize(4);
        Ground_z[0] = Ground.packet[0];
        Ground_z[1] = Ground.packet[1];
        Ground_z[2] = Ground.packet[2];
        Ground_z[3] = Ground.packet[3];
        dist[0] = Ground_z[0] - Leg_z[0];
        dist[1] = Ground_z[1] - Leg_z[1];
        dist[2] = Ground_z[2] - Leg_z[2];
        dist[3] = Ground_z[3] - Leg_z[3];
        avg = (dist[0] + dist[1] + dist[2] + dist[3])/4;
        error.packet[0] = dist[0] - avg;
        error.packet[1] = dist[1] - avg;
        error.packet[2] = dist[2] - avg;
        error.packet[3] = dist[3] - avg;

        //check///////////////////////////////////////////////////////////////////////////////
        //printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
        //ROS_INFO("[Error1,Error2,Error3,Error4]: %f,%f,%f,%f",error.packet[0],error.packet[1],error.packet[2],error.packet[3]);
        //ROS_INFO("[z1,z2,z3,z4]: %f,%f,%f,%f",dist[0],dist[1],dist[2],dist[3]);
        pub_.publish(error);
    }

    void callback2(const rplidar_data::packet& Leg)
    {
        error.packet.resize(4);
        Leg_z[0] = Leg.packet[0];
        Leg_z[1] = Leg.packet[1];
        Leg_z[2] = Leg.packet[2];
        Leg_z[3] = Leg.packet[3];

        dist[0] = Ground_z[0] - Leg_z[0];
        dist[1] = Ground_z[1] - Leg_z[1];
        dist[2] = Ground_z[2] - Leg_z[2];
        dist[3] = Ground_z[3] - Leg_z[3];

        avg = (dist[0] + dist[1] + dist[2] + dist[3])/4;

        error.packet[0] = dist[0] - avg;
        error.packet[1] = dist[1] - avg;
        error.packet[2] = dist[2] - avg;
        error.packet[3] = dist[3] - avg;

        //check///////////////////////////////////////////////////////////////////////////////
        printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
        ROS_INFO("[Error1,Error2,Error3,Error4]: %f,%f,%f,%f",error.packet[0],error.packet[1],error.packet[2],error.packet[3]);
        pub_.publish(error);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_1;
    ros::Subscriber sub_2;
    rplidar_data::packet error;
};


int main(int argc, char **argv)
{
    Ground_z[0] = 200.0;
    Ground_z[1] = 200.0;
    Ground_z[2] = 200.0;
    Ground_z[3] = 200.0;
    Leg_z[0] = 17.7;
    Leg_z[1] = 17.7;
    Leg_z[2] = 17.7;
    Leg_z[3] = 17.7;
    avg = 182.3;
    printf("\n");
    printf("\n");
    printf("Distance error can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output1 : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output2 : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****z_dist_error node*****\n""\x1b[0m");
    ros::init(argc, argv, "z_dist_error");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
