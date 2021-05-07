#include "ros/ros.h" 
#include "rplidar_data/xyz.h"
#include "rplidar_data/alpha.h"
#include "math.h"

class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
	  {
	    pub_ = n_.advertise<rplidar_data::alpha>("/controled_theta",1);
	    sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
	  }

        void callback(const rplidar_data::xyz& xyz)
	  {
	    int count = xyz.count;
	    double z_sum = 0.0;
	    int z_count = 0;
	    for(int i = 0; i<count; i++)
	      {
		if(xyz.z[i]<1000.0)
		  {
		    z_sum = z_sum + xyz.z[i];
		    z_count++;
		  }
	      }
	    double d = z_sum/(z_count+0.001); // If z_count is zero, theta.alpha is to be Inf. So, 0.001 can prevent this.
	    rplidar_data::alpha theta;
	    theta.alpha = atan(40/sqrt(0.4*0.4+d*d));
	    pub_.publish(theta);
	  }

private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    printf("\n");
    printf("\n");
    printf("Controled theta can be calculated in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /xyz\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****control_theta node*****\n""\x1b[0m");
    ros::init(argc, argv, "control_theta");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }
 
    return 0;
}
