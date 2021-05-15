#include "ros/ros.h" 
#include "rplidar_data/alpha.h"
#include "rplidar_data/time.h"

#define Ki 3 //Integral gain
#define Limit_value 3333.3

double dt;
double Sum;
double dt_prev;
double dt_now;
rplidar_data::time t;
int count = 0;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
	pub_ = n_.advertise<rplidar_data::alpha>("/Ki_error4",1);
	sub_ = n_.subscribe("/error4",1,&SubscribeAndPublish::callback,this);
	}
	
	void callback(const rplidar_data::alpha& error)
	{
	  if (count == 0)
	    {
	      t.time = ros::Time::now();
	      int t_sec = t.time.sec;
	      double t_nsec = t.time.nsec/1000000000.0;
	      dt_prev = t_sec+t_nsec;
	      count++;
	      return;
	    }
	  t.time = ros::Time::now();
	  int t_sec = t.time.sec;
	  double t_nsec = t.time.nsec/1000000000.0;
	  dt_now = t_sec+t_nsec;
	  dt = dt_now - dt_prev;
	  double e = error.alpha;
	  Sum += e*dt;
	  dt_prev = dt_now;
	  rplidar_data::alpha output;
          if (Sum>Limit_value)
          {
              Sum = Limit_value;
          }
          if (Sum<-Limit_value)
          {
              Sum = -Limit_value;
          }
	  output.alpha = Ki*Sum;
	  pub_.publish(output);	
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};


int main(int argc, char **argv)
{
    Sum = 0.0;
    printf("\n");
    printf("\n");
    printf("This is integral controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /alpha\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****integral_controller4*****\n""\x1b[0m");
    ros::init(argc, argv, "integral_controller4");
    SubscribeAndPublish NH;
    t.time = ros::Time::now();
    int A = t.time.sec;
    double B = t.time.nsec/1000000000.0;
    ROS_INFO("Time checking: %f",A+B);
    while(ros::ok())
    {
    ros::spinOnce();
    }
    return 0;
}
