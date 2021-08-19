#include "ros/ros.h" 
#include "rplidar_data/xyz.h"
#include "rplidar_data/packet.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Dense>
#include "math.h"
using namespace std;
#define mapsize 40 //one direction -> 2*mapsize+1
#define stepsize 9 //divisor of 2*mapsize+1
#define initial_value 1 //initial value of map
#define show 0 // show : 1, not show : 0

double local_map[(2*mapsize+1)/stepsize][(2*mapsize+1)/stepsize];

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub_ = n_.advertise<rplidar_data::packet>("/Ground_z",1);
        sub_ = n_.subscribe("/xyz",1,&SubscribeAndPublish::callback,this);
    }

    void callback(const rplidar_data::xyz& input)
    {
        int t = ((2*mapsize+1)/stepsize-1)/2;
        Eigen::Matrix<float,3,1> b;
        A(0,0) = ((2*mapsize+1)/stepsize)*((2*mapsize+1)/stepsize); A(0,1) = 0.0; A(0,2) = 0.0;
        A(1,0) = 0.0; A(1,1) = ((2*mapsize+1)/stepsize)*2*(stepsize*stepsize*t*(t+1)*(2*t+1)/6); A(1,2) = 0.0;
        A(2,0) = 0.0; A(2,1) = 0.0; A(2,2) = ((2*mapsize+1)/stepsize)*2*(stepsize*stepsize*t*(t+1)*(2*t+1)/6);

        double Sum_Sr = 0.0;
        double Sum_St = 0.0;
        double H = 0.0;
        double XZ = 0.0;
        double YZ = 0.0;
        double avg_z = 0.0;

        //local_mapping
        for(int i = 0; i<input.count;++i)
        {
            if(input.x[i]<mapsize+1 && input.y[i]<mapsize+1 && input.x[i]>-(mapsize+1) && input.y[i]>-(mapsize+1))
            {
                for(int xx = 0;xx<(2*mapsize+1)/stepsize;++xx)
                {
                    for(int yy = 0;yy<(2*mapsize+1)/stepsize;++yy)
                    {

                        if(-mapsize+stepsize*xx <= input.x[i] && input.x[i] < -mapsize+stepsize*(xx+1) && -mapsize+stepsize*yy <= input.y[i] && input.y[i] < -mapsize+stepsize*(yy+1))
                        {
                            local_map[xx][yy] = input.z[i];
                        }
                    }
                }
            }
        }

        //smoothing
        for(int i = 0;i<(2*mapsize+1)/stepsize;++i)
        {
            for(int j = 0;j<(2*mapsize+1)/stepsize;++j)
            {
                if(local_map[i][j] == 1)
                {
                    int local_sum = 0;
                    int local_count = 0;
                    if(i != 0)
                    {
                        local_sum = local_sum + local_map[i-1][j];
                        ++local_count;
                    }
                    if(i != (2*mapsize+1)/stepsize-1)
                    {
                        local_sum = local_sum + local_map[i+1][j];
                        ++local_count;
                    }
                    if(j != 0)
                    {
                        local_sum = local_sum + local_map[i][j-1];
                        ++local_count;
                    }
                    if(j != (2*mapsize+1)/stepsize-1)
                    {
                        local_sum = local_sum + local_map[i][j+1];
                        ++local_count;
                    }
                    if(i != 0 && j != 0)
                    {
                        local_sum = local_sum + local_map[i-1][j-1];
                        ++local_count;
                    }
                    if(i != 0 && j != (2*mapsize+1)/stepsize-1)
                    {
                        local_sum = local_sum + local_map[i-1][j+1];
                        ++local_count;
                    }
                    if(i != (2*mapsize+1)/stepsize-1 && j != 0)
                    {
                        local_sum = local_sum + local_map[i+1][j-1];
                        ++local_count;
                    }
                    if(i != (2*mapsize+1)/stepsize-1 && j != (2*mapsize+1)/stepsize-1)
                    {
                        local_sum = local_sum + local_map[i+1][j+1];
                        ++local_count;
                    }
                    local_map[i][j] = (int)(local_sum/local_count);
                    if(local_map[i][j]==0)
                    {
                        ROS_INFO("%d,%d",local_sum,local_count);
                    }
                }
            }
        }

        //calculate b (Ax = b) , calculate avg_z
        for(int i=0; i<(2*mapsize+1)/stepsize;i++)
        {
            for(int j = 0;j<(2*mapsize+1)/stepsize;j++)
            {
                H = H + local_map[i][j];
                XZ = XZ + (stepsize*i-mapsize+(stepsize-1)/2)*local_map[i][j];
                YZ = YZ + (stepsize*j-mapsize+(stepsize-1)/2)*local_map[i][j];
                avg_z = avg_z + local_map[i][j];
            }
        }

        b(0,0) = H; b(1,0) = XZ; b(2,0) = YZ;
        avg_z = avg_z/((2*mapsize+1)*(2*mapsize+1));

        //calculate coefficient
        double a0 = (A.inverse()*b)(0,0);
        double a1 = (A.inverse()*b)(1,0);
        double a2 = (A.inverse()*b)(2,0);
        //ROS_INFO("a0,a1,a2 : %f,%f,%f",a0,a1,a2);

        //calculate Sr, St
        for(int i=0; i<(2*mapsize+1)/stepsize;++i)
        {
            for(int j = 0;j<(2*mapsize+1)/stepsize;++j)
            {
                double SR = pow((local_map[i][j] - a0 - a1*(stepsize*i-mapsize+(stepsize-1)/2) - a2*(stepsize*j-mapsize+(stepsize-1)/2)),2.0);
                double St = pow((local_map[i][j] - avg_z),2.0);
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
                    printf("%d, ",(int)local_map[((2*mapsize+1)/stepsize)-1-i][j]);
                }
                printf("\n");
            }
            printf("******************************************\n");
        }

        //(St-Sr)/St = r^2 ( 0 < r^2 < 1 )
        rplidar_data::packet z;
        z.packet.resize(4);
        if(stepsize==1)
        {
            z.packet[0] = local_map[62][18];
            z.packet[1] = local_map[18][18];
            z.packet[2] = local_map[18][62];
            z.packet[3] = local_map[62][62];
        }
        if(stepsize==3)
        {
            z.packet[0] = local_map[20][6];
            z.packet[1] = local_map[6][6];
            z.packet[2] = local_map[6][20];
            z.packet[3] = local_map[20][20];
        }
        if(stepsize==9)
        {

            z.packet[0] = local_map[7][2];
            z.packet[1] = local_map[2][2];
            z.packet[2] = local_map[2][7];
            z.packet[3] = local_map[7][7];
        }
        if(show==0)
        {
            printf("%f",z.packet[0]);
            printf("\n");
            printf("%f",z.packet[1]);
            printf("\n");
            printf("%f",z.packet[2]);
            printf("\n");
            printf("%f",z.packet[3]);
            printf("\n");
        }
        double error = (Sum_St-Sum_Sr)/Sum_St;
        pub_.publish(z);
        ROS_INFO("r^2 = %f[Coefficient of Determination,1(good)]",error);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    Eigen::Matrix3f A;
};


int main(int argc, char **argv)
{

    for(int j=0; j<(2*mapsize+1)/stepsize;++j)
    {
        for(int i=0; i<(2*mapsize+1)/stepsize;++i)
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
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
