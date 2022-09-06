//这个函数用来求解雷达和IMU之间的外参，其实是应该包含了六个变量，但是实际上求解可能会有些问题，没有问题，可以用来进行标定
//只要给定的位姿变换没有问题，也就是雷达的里程计和IMU的里程计没问题，求出来的就没有问题，基本上很准确
#include "ros/ros.h"
#include <SolveAXXB/conventionalaxxbsvdsolver.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iostream>
#include <string>
using namespace std;
class Test
{
public:
	bool flag1;
	bool flag2;
	ros::NodeHandle nh;
	ros::Subscriber sublegodom ;
	ros::Subscriber sublidarodom ;
	Eigen::Quaterniond q_wlidar_curr;    //雷达当前的里程计
	Eigen::Vector3d t_wlidar_curr;
	Eigen::Quaterniond q_wleg_curr;    //腿部里程计其实是根据IMU得到的
	Eigen::Vector3d t_wleg_curr;
	Poses A,B;
	Pose p1,p2;
public:
	Test()
	{
		sublegodom = nh.subscribe<nav_msgs::Odometry>("/leg_odom", 100,  &Test::legodomHandler,this);
		sublidarodom = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100, &Test::lidarodomHandle,this);
		for (int i = 0; i < 3; i++)
		{
			t_wlidar_curr[i] = 0;
			t_wleg_curr[i] = 0;
		}
		flag1 = false;
		flag2 = false;
		q_wlidar_curr.x() = 0;
		q_wlidar_curr.y() = 0;
		q_wlidar_curr.z() = 0;
		q_wlidar_curr.w() = 1; 

		q_wleg_curr.x() = 0;
		q_wleg_curr.y() = 0;
		q_wleg_curr.z() = 0;
		q_wleg_curr.w() = 1; // 一定要初始化

	}
	void lidarodomHandle(const nav_msgs::Odometry::ConstPtr &lidarOdometry)
	{
		

		q_wlidar_curr.x() = lidarOdometry->pose.pose.orientation.x;      //得到雷达低频里程计数据当前值
		q_wlidar_curr.y() = lidarOdometry->pose.pose.orientation.y;
		q_wlidar_curr.z() = lidarOdometry->pose.pose.orientation.z;
		q_wlidar_curr.w() = lidarOdometry->pose.pose.orientation.w;
		t_wlidar_curr.x() = lidarOdometry->pose.pose.position.x;
		t_wlidar_curr.y() = lidarOdometry->pose.pose.position.y;
		t_wlidar_curr.z() = lidarOdometry->pose.pose.position.z;
		flag1=true;
		if(flag1==true&&flag2==true)
		{
			p1.topLeftCorner(3,3) = q_wlidar_curr.toRotationMatrix();
			p1(0,3) = t_wlidar_curr.x();
			p1(1,3) = t_wlidar_curr.y(); 
			p1(2,3) = t_wlidar_curr.z(); 
			p1(3,3) = 1;
			A.push_back(p1);
			B.push_back(p2);
			flag1=false;
			flag2=false;
		}
		//cout<<"A.size():"<<A.size()<<endl;    //最后的求解结果为  很准，确实很准
	  // 	0.999803 0.000169862   0.0198246    0.021343
     //     3.0304e-05    0.999949  -0.0100961 -0.00483625
     //    -0.0198253   0.0100947    0.999752    0.273546
     //     0           0           0           1

		if(A.size()==150)   
		{
			ConventionalAXXBSVDSolver e1(A,B);    
			Pose ret = e1.SolveX();
			std::cout<<ret<<endl;
		}
	}

	void legodomHandler(const nav_msgs::Odometry::ConstPtr &legOdometry)
	{
		

		// high frequence publish
		Eigen::Quaterniond q_wodom_curr;
		Eigen::Vector3d t_wodom_curr;
		q_wleg_curr.x() = legOdometry->pose.pose.orientation.x;      
		q_wleg_curr.y() = legOdometry->pose.pose.orientation.y;
		q_wleg_curr.z() = legOdometry->pose.pose.orientation.z;
		q_wleg_curr.w() = legOdometry->pose.pose.orientation.w;
		t_wleg_curr.x() = legOdometry->pose.pose.position.x;
		t_wleg_curr.y() = legOdometry->pose.pose.position.y;
		t_wleg_curr.z() = legOdometry->pose.pose.position.z;
		p2.topLeftCorner(3,3) = q_wleg_curr.toRotationMatrix();
		p2(0,3) = t_wleg_curr.x();
		p2(1,3) = t_wleg_curr.y(); 
		p2(2,3) = t_wleg_curr.z(); 
		p2(3,3) = 1;
		flag2 = true;
	}


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "Fusion");

    Test test;

    ROS_INFO("\033[1;32m---->\033[0m Test Started.");

    ros::spin();

    return 0;
    
}