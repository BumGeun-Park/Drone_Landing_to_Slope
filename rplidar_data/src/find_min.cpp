#include <iostream>
#include "ros/ros.h"
#include "rplidar_data/xyz.h"
#include "rplidar_data/packet.h"
#include "math.h"

#define length 11.25
#define z_bias 0.001

#define x1 +length
#define y1 -length

#define x2 -length
#define y2 -length

#define x3 -length
#define y3 +length

#define x4 +length
#define y4 +length

double dist1;
double dist2;
double dist3;
double dist4;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_ = n_.advertise<rplidar_data::packet>("/Ground_z",1);
	sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const rplidar_data::xyz& xyz)
	{
		int count = xyz.count;
                z.packet.resize(4);
                for(int i = 0; i<count; ++i)
		  {
                    double dist_1 = sqrt(pow((x1-xyz.x[i]),2)+pow((y1-xyz.y[i]),2));
                    double dist_2 = sqrt(pow((x2-xyz.x[i]),2)+pow((y2-xyz.y[i]),2));
                    double dist_3 = sqrt(pow((x3-xyz.x[i]),2)+pow((y3-xyz.y[i]),2));
                    double dist_4 = sqrt(pow((x4-xyz.x[i]),2)+pow((y4-xyz.y[i]),2));
                    if(dist_1<dist1)
		      {
                        z.packet[0] = xyz.z[i];
                        dist1 = dist_1;

                        //check///////////////////////////////////////////////////////////////////////////////
                        printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
                        ROS_INFO("[x1,y1,z1] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
		      }
                    if(dist_2<dist2)
                      {
                        z.packet[1] = xyz.z[i];
                        dist2 = dist_2;

                        //check///////////////////////////////////////////////////////////////////////////////
                        printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
                        ROS_INFO("[x2,y2,z2] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
                      }
                    if(dist_3<dist3)
                      {
                        z.packet[2] = xyz.z[i];
                        dist3 = dist_3;

                        //check///////////////////////////////////////////////////////////////////////////////
                        printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
                        ROS_INFO("[x3,y3,z3] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
                      }
                    if(dist_4<dist4)
                      {
                        z.packet[3] = xyz.z[i];
                        dist4 = dist_4;

                        //check///////////////////////////////////////////////////////////////////////////////
                        printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
                        ROS_INFO("[x4,y4,z4] = [%f,%f,%f]", xyz.x[i], xyz.y[i], xyz.z[i]);
                      }
		  }
		pub_.publish(z);
                printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
                ROS_INFO("[dist1,dist2,dist3,dist4] = [%f,%f,%f,%f]",dist1,dist2,dist3,dist4);
		dist1 = dist1+z_bias;
                dist2 = dist2+z_bias;
                dist3 = dist3+z_bias;
                dist4 = dist4+z_bias;
	}


private:
	ros::NodeHandle n_;
        ros::Publisher pub_;
	ros::Subscriber sub_;
	rplidar_data::xyz xyz;
        rplidar_data::packet z;
};

int main(int argc, char **argv)
{
    dist1 = 100.0;
    dist2 = 100.0;
    dist3 = 100.0;
    dist4 = 100.0;
    printf("\n");
    printf("\n");
    printf("Minimum distance data can be chosen in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /xyz\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /packet\n""\x1b[0m");
    printf("\n");
    printf("*****find_min node*****\n""\x1b[0m");
    ros::init(argc, argv, "find_min");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}
