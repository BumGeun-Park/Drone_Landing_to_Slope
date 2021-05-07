#include "ros/ros.h"
#include "rplidar_data/polar.h"
#include "rplidar_data/xyz.h"
#include "rplidar_data/phi.h"
#include "rplidar_data/alpha.h"
#include "math.h"
#define DEG2RAD(x) ((x)*M_PI/180)
#define detect_angle 20 // (deg)
double phi = 0.0;
double Theta = DEG2RAD(detect_angle);

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::xyz>("/xyz",1);
	sub_1 = n_.subscribe("/polar",1,&SubscribeAndPublish::callback1,this);
	sub_2 = n_.subscribe("/phi",1,&SubscribeAndPublish::callback2,this);
	sub_3 = n_.subscribe("/controled_theta",1,&SubscribeAndPublish::callback3,this);
	}

	void callback1(const rplidar_data::polar& input1)
	{
	  int count = input1.count;
	  int new_count = 0;
	  int t = 0;
	  for(int i = 0; i<count ; i++)
	    {
		if(input1.radian[i] < Theta & input1.radian[i] > -Theta)
		  {
			new_count++;
		  }
	    }
	  xyz.x.resize(new_count);
	  xyz.y.resize(new_count);
	  xyz.z.resize(new_count);
	  xyz.count = new_count;
	  for(int i = 0; i<count ; i++)
	    {
		if(input1.radian[i] < Theta & input1.radian[i] > -Theta)
		  {
			xyz.x[t] = 100*(-input1.radius[i])*sin(input1.radian[i]);
			xyz.y[t] = 100*input1.radius[i]*cos(input1.radian[i])*sin(phi);
			xyz.z[t] = 100*input1.radius[i]*cos(input1.radian[i])*cos(phi);
			t++;
                        //ROS_INFO("[x,y,z] = [%f,%f,%f],[cm]" , xyz.x[t], xyz.y[t],xyz.z[t]);
		  }
	    }
	  //ROS_INFO("[x,y,z] = [%f,%f,%f],[cm]" , xyz.x, xyz.y,xyz.z);
	  pub_.publish(xyz);
	}

	void callback2(const rplidar_data::phi& input2)
	{
	phi = input2.phi;
	}

	void callback3(const rplidar_data::alpha& theta)
	{
	Theta = theta.alpha;
	}

private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_1;
	ros::Subscriber sub_2;
	ros::Subscriber sub_3;
	rplidar_data::xyz xyz;
};


int main(int argc, char **argv)
{
    printf("\n");
    printf("\n");
    printf("Sperical coordinate can be transformed to cartesian coordinate in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input1 : /polar\n""\x1b[0m");
    printf("\x1b[31m""Input2 : /phi\n""\x1b[0m");
    printf("\x1b[31m""Input3 : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /xyz\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****assemble node*****\n""\x1b[0m");
    ros::init(argc, argv, "assemble");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
