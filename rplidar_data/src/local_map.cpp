#include "ros/ros.h" 
#include "rplidar_data/xyz.h"
#include "rplidar_data/alpha.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include "math.h"
using namespace std;
#define mapsize 40 //one direction -> 2*mapsize+1
#define stepsize 9 //divisor of 2*mapsize+1
#define initial_value 1 //initial value of map
#define show 1 // show : 1, not show : 0

double local_map[2*mapsize+1][2*mapsize+1];
double scaled_map[(2*mapsize+1)/stepsize][(2*mapsize+1)/stepsize];

class SubscribeAndPublish
{
public:
        SubscribeAndPublish()
        {
        pub_ = n_.advertise<rplidar_data::alpha>("/standard_error",1);
        sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
        }

        void callback(const rplidar_data::xyz& input)
        {
            Eigen::Matrix<float,3,1> b;
	    A(0,0) = (2*mapsize+1)*(2*mapsize+1); A(0,1) = 0.0; A(0,2) = 0.0;
	    A(1,0) = 0.0; A(1,1) = (2*mapsize+1)*2*(mapsize*(mapsize+1)*(2*mapsize+1)/6); A(1,2) = 0.0;
	    A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = (2*mapsize+1)*2*(mapsize*(mapsize+1)*(2*mapsize+1)/6);

	    double Sum_Sr = 0.0;
	    double Sum_St = 0.0;
	    double H = 0.0;
            double XZ = 0.0;
            double YZ = 0.0;
	    double avg_z = 0.0;

//local_mapping
	    for(int i = 0; i<input.count;i++)
		{
		   if(input.x[i]<mapsize+1&input.y[i]<mapsize+1&input.x[i]>-(mapsize+1)&input.y[i]>-(mapsize+1))
			{
			  local_map[(int)input.x[i]+mapsize][(int)input.y[i]+mapsize] = input.z[i];
			  //ROS_INFO("%d,%d",input.x[i]+40,input.y[i]+40);
			  //ROS_INFO("%d,%d,%f",input.x[i]+40,input.y[i]+40,local_map[input.x[i]+40][input.y[i]+40]);
			}
		}

//map scaling
	    for(int i=0; i<(2*mapsize+1)/stepsize;i++)
		{
    		  for(int j = 0;j<(2*mapsize+1)/stepsize;j++)
    		    {
			double num = 0;
			double local_sum = 0.0;
			for(int kx = 0; kx<stepsize; kx++)
			  {
			    for(int ky = 0; ky<stepsize; ky++)
			      {
				if(local_map[stepsize*i+kx][stepsize*j+ky] != initial_value)
			          {
				    local_sum = local_sum+local_map[stepsize*i+kx][stepsize*j+ky];
				    num++;
			      	  }
			      }
			  }
			if(num!=0)
			  {
			    scaled_map[i][j] = local_sum/num;
			  }
			else
			  scaled_map[i][j] = initial_value;
    		    }
		}

//calculate b (Ax = b) , calculate avg_z
	    for(int i=0; i<(2*mapsize+1)/stepsize;i++)
		{
    		  for(int j = 0;j<(2*mapsize+1)/stepsize;j++)
    		    {
                      H = H + scaled_map[i][j];
                      XZ = XZ + (stepsize*i-mapsize+(stepsize-1)/2)*scaled_map[i][j];
                      YZ = YZ + (stepsize*j-mapsize+(stepsize-1)/2)*scaled_map[i][j];
		      avg_z = avg_z + scaled_map[i][j];
    		    }
		}

	    b(0,0) = H; b(1,0) = XZ; b(2,0) = YZ;
	    avg_z = avg_z/((2*mapsize+1)*(2*mapsize+1));
	    //ROS_INFO("avg_z: %f",avg_z);

//calculate coefficient
            double a0 = (A.inverse()*b)(0,0);
	    double a1 = (A.inverse()*b)(1,0);
	    double a2 = (A.inverse()*b)(2,0);
	    //ROS_INFO("a0,a1,a2 : %f,%f,%f",a0,a1,a2);

//calculate Sr, St
	    for(int i=0; i<(2*mapsize+1)/stepsize;i++)
		{
    		  for(int j = 0;j<(2*mapsize+1)/stepsize;j++)
    		    {
			double SR = pow((scaled_map[i][j] - a0 - a1*(stepsize*i-mapsize+(stepsize-1)/2) - a2*(stepsize*j-mapsize+(stepsize-1)/2)),2.0);
		        double St = pow((scaled_map[i][j] - avg_z),2.0);
			//ROS_INFO("SR,St:%f,%f",SR,St);
			//ROS_INFO("SR,St:%f,%f",a0 + a1*(i-40.0) + a2*(j-40.0),avg_z);
			Sum_Sr = Sum_Sr + SR;
		        Sum_St = Sum_St + St;
    		    }
		}
		//ROS_INFO("SR,St:%f,%f",Sum_Sr,Sum_St);
		//ROS_INFO("a0,avg_z:%f,%f",a0,avg_z);

//Show map(2D array)
if(show==1)
{
	    ROS_INFO("start\n");
            for(int i=0; i<(2*mapsize+1)/stepsize;++i)
	      {
                for(int j=0; j<(2*mapsize+1)/stepsize;++j)
		  {
                    printf("%d, ",(int)scaled_map[((2*mapsize+1)/stepsize)-1-i][j]);
		  }
		printf("\n");
	      }
	    printf("******************************************\n");
}

//(St-Sr)/St = r^2 ( 0 < r^2 < 1 )
	    rplidar_data::alpha error;
	    error.alpha = (Sum_St-Sum_Sr)/Sum_St;
	    //error.alpha = sqrt(Sum_Sr/(81*81-3));
	    pub_.publish(error);
        ROS_INFO("r^2 = %f[Coefficient of Determination,1(good)]",error.alpha);
        }
private:
        ros::NodeHandle n_;
        ros::Publisher pub_;
        ros::Subscriber sub_;
	Eigen::Matrix3f A;
};


int main(int argc, char **argv)
{

    for(int j=0; j<2*mapsize+1;j++)
      {
        for(int i=0; i<2*mapsize+1;i++)
          {
            local_map[i][j] = initial_value;
          }
        printf("\n");
      }
    printf("\n");
    printf("\n");
    printf("Standard error of landing plane can be calculated so that we can choose wheater drone will be landed to plane in this node!\n");
    printf("\n");
    printf("made by Bum Geun Park, 2021.04\n");
    printf("\n");
    printf("\x1b[31m""Input : /xyz\n""\x1b[0m");
    printf("\n");
    printf("\x1b[34m""Output : /standard_error\n""\x1b[0m");
    printf("\n");
    printf("\x1b[37m""*****local_map node*****\n""\x1b[0m");
    /*cout<<"\n"<<local_map[0][0]<<endl;
    cout<<"\n"<<local_map[2][3]<<endl;
    cout<<"\n"<<local_map[10][5]<<endl;*/

    ros::init(argc, argv, "local_map");
    SubscribeAndPublish NH;
    while(ros::ok())
    {
    ros::spinOnce();
    }

    return 0;
}
