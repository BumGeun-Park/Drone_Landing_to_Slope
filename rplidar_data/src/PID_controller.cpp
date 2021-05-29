#include "ros/ros.h" 
#include "rplidar_data/packet.h"
#include "rplidar_data/time.h"
#include "sensor_msgs/JointState.h"

#define Ki 1 //Integral gain
#define Kp 1 //Proportional gain
#define Kd 1 //Derivative gain
#define Limit_value 500

#define Leg1 1
#define Leg2 2
#define Leg3 3
#define Leg4 4
#define Arm_length 10

#define DEG2RAD(x) ((x)*M_PI/180)
#define upper_limit DEG2RAD(46) // 46 deg
#define lower_limit DEG2RAD(-10) // -10 deg

double dt;
double Sum1;
double Sum2;
double Sum3;
double Sum4;
double dt_prev;
double dt_now;

double theta[4];
double theta_dot[4];

double error_dot[4];

double Roll_alpha;
double Pitch_alpha;

rplidar_data::time t;
int count = 0;

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{
        pub_ = n_.advertise<rplidar_data::packet>("/feedback",1);
        sub_ = n_.subscribe("/error",1,&SubscribeAndPublish::callback1,this);
        sub_2 = n_.subscribe("/dynamixel_workbench/joint_states",1,&SubscribeAndPublish::callback2,this);
        sub_3 = n_.subscribe("/alpha_RP",1,&SubscribeAndPublish::callback3,this);
	}
	
        void callback1(const rplidar_data::packet& error)
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

          // derivative_error
          error_dot[0] = -10/(cos(theta[0])*cos(theta[0]))*theta_dot[0];
          error_dot[1] = -10/(cos(theta[1])*cos(theta[1]))*theta_dot[1];
          error_dot[2] = -10/(cos(theta[2])*cos(theta[2]))*theta_dot[2];
          error_dot[3] = -10/(cos(theta[3])*cos(theta[3]))*theta_dot[3];
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
          //ROS_INFO("sum1,sum2,sum3,sum4: %f,%f,%f,%f",Sum1,Sum2,Sum3,Sum4); // I_error
          //ROS_INFO("sum1,sum2,sum3,sum4: %f,%f,%f,%f",error.packet[0],error.packet[1],error.packet[2],error.packet[3]); // P_error
          //ROS_INFO("sum1,sum2,sum3,sum4: %f,%f,%f,%f",error_dot[0],error_dot[1],error_dot[2],error_dot[3]); // // D_error

          output.packet[0] = Ki*Sum1 + Kp*error.packet[0] + Kd*error_dot[0];
          output.packet[1] = Ki*Sum2 + Kp*error.packet[1] + Kd*error_dot[1];
          output.packet[2] = Ki*Sum3 + Kp*error.packet[2] + Kd*error_dot[2];
          output.packet[3] = Ki*Sum4 + Kp*error.packet[3] + Kd*error_dot[3];

          //safe
          if(theta[0]<lower_limit||theta[0]>upper_limit)
          {
              output.packet[0] = -1000*abs(theta[0])/theta[0];
              ROS_INFO("\x1b[31m""Warning Leg 1""\x1b[31m");
          }
          if(theta[1]<lower_limit||theta[1]>upper_limit)
          {
              output.packet[1] = -1000*abs(theta[1])/theta[1];
              ROS_INFO("\x1b[31m""Warning Leg 2""\x1b[31m");
          }
          if(theta[2]<lower_limit||theta[2]>upper_limit)
          {
              output.packet[2] = -1000*abs(theta[2])/theta[2];
              ROS_INFO("\x1b[31m""Warning Leg 3""\x1b[31m");
          }
          if(theta[3]<lower_limit||theta[3]>upper_limit)
          {
              output.packet[3] = -1000*abs(theta[3])/theta[3];
              ROS_INFO("\x1b[31m""Warning Leg 4""\x1b[31m");
          }

          //check///////////////////////////////////////////////////////////////////////////////
          printf("\x1b[34m""[checking operation(polar coordinate)]""\x1b[0m");
          ROS_INFO("[Error]: %f,%f,%f,%f",output.packet[0],output.packet[1],output.packet[2],output.packet[3]);
	  pub_.publish(output);	
	}

        void callback2(const sensor_msgs::JointState& input)
        {
            theta[0] = input.position[Leg1];
            theta_dot[0] = input.velocity[Leg1];
            theta[1] = input.position[Leg2];
            theta_dot[1] = input.velocity[Leg2];
            theta[2] = input.position[Leg3];
            theta_dot[2] = input.velocity[Leg3];
            theta[3] = input.position[Leg4];
            theta_dot[3] = input.velocity[Leg4];
        }
        void callback3(const rplidar_data::packet& input)
        {
            Roll_alpha = input.packet[0];
            Pitch_alpha = input.packet[1];
        }
private:
	ros::NodeHandle n_;
	ros::Publisher pub_;
	ros::Subscriber sub_;
        ros::Subscriber sub_2;
        ros::Subscriber sub_3;
};

int main(int argc, char **argv)
{
    Sum1 = 0.0;
    Sum2 = 0.0;
    Sum3 = 0.0;
    Sum4 = 0.0;
    dt = 0.0;
    printf("\n");
    printf("\n");
    printf("This is PID_controller node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.05\n");
    printf("\n");
    printf("\x1b[31m""Input : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /packet\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****PID_controller*****\n""\x1b[0m");
    ros::init(argc, argv, "PID_controller");
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
