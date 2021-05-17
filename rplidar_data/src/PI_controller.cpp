#include "ros/ros.h" 
#include "rplidar_data/packet.h"
#include "rplidar_data/time.h"

#define Ki 3 //Integral gain
#define Kp 3 //Proportional gain
#define Limit_value 3333.3

double dt;
double Sum1;
double Sum2;
double Sum3;
double Sum4;
double dt_prev;
double dt_now;
rplidar_data::time t;
int count = 0;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_ = n_.advertise<rplidar_data::packet>("/feedback",1);
        sub_ = n_.subscribe("/error",1,&SubscribeAndPublish::callback,this);
	}
	
        void callback(const rplidar_data::packet& error)
	{
          // calculate time derivative
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
          //

          double e1 = error.packet[0];
          double e2 = error.packet[1];
          double e3 = error.packet[2];
          double e4 = error.packet[3];
          Sum1 += e1*dt;
          Sum2 += e2*dt;
          Sum3 += e3*dt;
          Sum4 += e4*dt;
	  dt_prev = dt_now;
          rplidar_data::packet output;
          output.packet.resize(4);

          // 1
          if (Sum1>Limit_value)
          {
              Sum1 = Limit_value;
          }
          if (Sum1<-Limit_value)
          {
              Sum1 = -Limit_value;
          }

          //2
          if (Sum2>Limit_value)
          {
              Sum2 = Limit_value;
          }
          if (Sum2<-Limit_value)
          {
              Sum2 = -Limit_value;
          }

          //3
          if (Sum3>Limit_value)
          {
              Sum3 = Limit_value;
          }
          if (Sum3<-Limit_value)
          {
              Sum3 = -Limit_value;
          }

          //4
          if (Sum4>Limit_value)
          {
              Sum4 = Limit_value;
          }
          if (Sum4<-Limit_value)
          {
              Sum4 = -Limit_value;
          }

          output.packet[0] = Ki*Sum1 + Kp*error.packet[0];
          output.packet[1] = Ki*Sum2 + Kp*error.packet[1];
          output.packet[2] = Ki*Sum3 + Kp*error.packet[2];
          output.packet[3] = Ki*Sum4 + Kp*error.packet[3];
          ROS_INFO("%f,%f,%f,%f",output.packet[0],output.packet[1],output.packet[2],output.packet[3]);
	  pub_.publish(output);	
	}
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
    Sum1 = 0.0;
    Sum2 = 0.0;
    Sum3 = 0.0;
    Sum4 = 0.0;
    printf("\n");
    printf("\n");
    printf("This is PI_controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****PI_controller*****\n""\x1b[0m");
    ros::init(argc, argv, "PI_controller");
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
