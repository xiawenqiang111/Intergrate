#include "ros/ros.h"
#include "tf/transform_datatypes.h" //转换函数头文件
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <cmath>
#include <chrono>
using namespace Eigen;
using namespace std;

class TransformFusion
{

private:
    ros::NodeHandle nh;

    ros::Publisher pubLaserOdometry2;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subLaserOdometry2;
    ros::Subscriber subOdomAftMapped;

    nav_msgs::Odometry laserOdometry2;
    float transformSum[6];
    float transformMapped[6];
    float transformlastleg[6];
    float transformAftMapped[6];
    float transformAftGtasm[6];
    float transformlastMap[6];
    float transformOut[6];
    deque<vector<float>> mapStore;
    vector<float> mapTemp;
    bool flag = false;
    bool flagGtasm = false;
    const int N = 200;      //数量会影响，不能太大 10
    double factor = 0.86;   //但是雷达频率的因子同样有影响，即使很小，影响外面
    double prop = 0.99;  
    double propo = 0.99;   //影响连续性，md影响本身的腿部数据 ，不能太大 0.3
    double rate = 0.05;

    Eigen::Quaterniond q_leg_last;
    Eigen::Vector3d t_leg_last;

    Eigen::Quaterniond q_leg_cur;
    Eigen::Vector3d t_leg_cur;

    Eigen::Quaterniond q_leg_inc;
    Eigen::Vector3d t_leg_inc;

    Eigen::Quaterniond q_leg_lidar;
    Eigen::Vector3d t_leg_lidar;

    Eigen::Quaterniond q_cur;
    Eigen::Vector3d t_cur;

    Eigen::Quaterniond q_temp;

    std_msgs::Header currentHeader;

public:
    TransformFusion()
    {

        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>("/integrate_init", 1000);
        subLaserOdometry =  nh.subscribe<nav_msgs::Odometry>("/leg_odom", 1000, &TransformFusion::laserOdometryHandler, this);
        subLaserOdometry2 = nh.subscribe<nav_msgs::Odometry>("/leg_odom_a", 1000, &TransformFusion::laserOdometryHandler2, this);
        subOdomAftMapped =  nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 10, &TransformFusion::odomAftMappedHandler, this);

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformMapped[i] = 0;
            transformlastleg[i] = 0;
            transformAftMapped[i] = 0;
            transformlastMap[i] = 0;
            transformAftGtasm[i] = 0;
        }
        for (int i = 0; i < 3; i++)
        {
            t_leg_lidar[i] = 0;
            t_leg_cur[i] = 0;
            t_leg_last[i] = 0;
            t_leg_inc[i] = 0;
            t_cur[i] = 0;
        }
        q_cur.x() = 0;
        q_cur.y() = 0;
        q_cur.z() = 0;
        q_cur.w() = 1; // 一定要初始化
        q_leg_last.x() = 0;
        q_leg_last.y() = 0;
        q_leg_last.z() = 0;
        q_leg_last.w() = 1; // 一定要初始化
       

        mapTemp.resize(6, 0);
    }

    void transformAssociate() //每一次都在进行腿部里程计的优化
    {
        // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(transformlastleg[0], Vector3d::UnitX()));
        // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(transformlastleg[1], Vector3d::UnitY()));
        // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(transformlastleg[2], Vector3d::UnitZ()));

       // q_leg_last = q_leg_cur;
        t_leg_last[0] = transformlastleg[3];
        t_leg_last[1] = transformlastleg[4];
        t_leg_last[2] = transformlastleg[5];

        // rollAngle = Eigen::AngleAxisd(transformSum[0], Vector3d::UnitX());
        // pitchAngle = Eigen::AngleAxisd(transformSum[1], Vector3d::UnitY());
        // yawAngle = Eigen::AngleAxisd(transformSum[2], Vector3d::UnitZ());

       // q_leg_cur = yawAngle * pitchAngle * rollAngle;
        t_leg_cur[0] = transformSum[3];
        t_leg_cur[1] = transformSum[4];
        t_leg_cur[2] = transformSum[5];

        //腿部增量
        q_leg_inc = q_leg_cur * q_leg_last.inverse();
        t_leg_inc = t_leg_cur - q_leg_inc * t_leg_last;
        // std::cout<<"t_leg_inc:"<<t_leg_inc[0]<<" "<<t_leg_inc[1]<<" "<<t_leg_inc[2]<<std::endl;

        //优化后的结果
        q_leg_lidar = q_leg_inc * q_cur;
        t_leg_lidar = t_leg_inc + q_leg_inc * t_cur;

        transformMapped[3] = t_leg_lidar[0];
        transformMapped[4] = t_leg_lidar[1];
        transformMapped[5] = t_leg_lidar[2];
        // std::cout<<"q_leg_last:"<<q_cur.coeffs()<<std::endl;
        // std::cout<<"t_leg_lidar:"<<t_leg_lidar[0]<<" "<<t_leg_lidar[1]<<" "<<t_leg_lidar[2]<<std::endl;

        Eigen::Vector3d eulerAngle = q_leg_lidar.matrix().eulerAngles(2, 1, 0);

        transformMapped[0] = eulerAngle[2]; // rpy,方法是没错的，原理也是对的
        transformMapped[1] = eulerAngle[1];
        transformMapped[2] = eulerAngle[0];
        // std::cout<<"transformMapped:"<<transformMapped[0]<<" "<<transformMapped[1]<<" "<<transformMapped[2]<<std::endl;

        if (flag == true)   //只要运行出现时间对不上
        {
            t_cur[0] = transformAftMapped[3] * factor + t_leg_lidar[0] * (1-factor);  //腿部数据
            t_cur[1] = transformAftMapped[4] * factor + t_leg_lidar[1] * (1-factor);
            t_cur[2] = transformAftMapped[5] * factor + t_leg_lidar[2] * (1-factor);

            // rollAngle = Eigen::AngleAxisd(transformAftMapped[0], Vector3d::UnitX());
            // pitchAngle = Eigen::AngleAxisd(transformAftMapped[1], Vector3d::UnitY());
            // yawAngle = Eigen::AngleAxisd(transformAftMapped[2], Vector3d::UnitZ());

            // Eigen::Quaterniond q_temp = yawAngle * pitchAngle * rollAngle;

            q_cur.x() = q_leg_lidar.x() * prop + q_temp.x() * (1-prop);
            q_cur.y() = q_leg_lidar.y() * prop + q_temp.y() * (1-prop);
            q_cur.z() = q_leg_lidar.z() * prop + q_temp.z() * (1-prop);
            q_cur.w() = q_leg_lidar.w() * prop + q_temp.w() * (1-prop);
        
           // q_cur = q_leg_lidar;
        }
        else
        {

            for (int i = 0; i < 3; i++)
            {
                t_cur[i] = t_leg_lidar[i];
            }
            q_cur = q_leg_lidar;
        }

        flag = false;
   
        for (int i = 3; i < 6; i++)
        {
            if(flagGtasm == false)
            {
                transformlastleg[i] = transformSum[i];
            }
            else {
                transformlastleg[i] = transformAftGtasm[i]*rate + transformSum[i]*(1-rate);
                flagGtasm = false ;
            }
            transformlastMap[i] = transformMapped[i];
            mapTemp[i] = transformlastMap[i]; //应该先给定大小
        }
        if (mapStore.size() < N)
        {
            mapStore.push_back(mapTemp);
            vector<float> vtmp = mapStore.back();
            for (int i = 3; i < 6; i++)
            {
                transformOut[i] = vtmp[i];
            }
        }
        else
        {
            for (int i = 3; i < 6; i++)
            {
                transformOut[i] = 0;
            }
            mapStore.pop_front();
            mapStore.push_back(mapTemp);

            for (int i = 0; i < N - 1; i++)
            {
                vector<float> vtmp = mapStore[i];
                for (int j = 3; j < 6; j++)
                {
                    transformOut[j] += vtmp[j]; //前9个值
                }
            }
            for (int i = 3; i < 6; i++)
            {
                transformOut[i] /= (N - 1);
                transformOut[i] = transformOut[i] * propo + mapTemp[i] * (1-propo);
            }
        }
       // std::cout<<"transformMapped:"<<transformOut[3]<<" "<<transformOut[4]<<" "<<transformOut[5]<<std::endl;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
    {
        currentHeader = laserOdometry->header;

        double roll, pitch, yaw;
        tf::Quaternion quat;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        // std::cout<<"geoQuat:"<<geoQuat.x<<" "<<geoQuat.y<<" "<<geoQuat.z<<" "<<geoQuat.w<<std::endl;
        tf::quaternionMsgToTF(geoQuat, quat);

        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        transformSum[0] = roll;
        transformSum[1] = pitch;
        transformSum[2] = yaw;

        q_leg_cur.x() = geoQuat.x;
        q_leg_cur.y() = geoQuat.y;
        q_leg_cur.z() = geoQuat.z;
        q_leg_cur.w() = geoQuat.w;

        //注意获取位置的顺序
        transformSum[3] = laserOdometry->pose.pose.position.x ;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;
        //std::cout<<"transformSum:"<<transformSum[3]<<" "<<transformSum[4]<<" "<<transformSum[5]<<std::endl;
          
        transformAssociate();
        
        q_leg_last = q_leg_cur;


        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = q_leg_lidar.x();
        laserOdometry2.pose.pose.orientation.y = q_leg_lidar.y();
        laserOdometry2.pose.pose.orientation.z = q_leg_lidar.z();
        laserOdometry2.pose.pose.orientation.w = q_leg_lidar.w();
        laserOdometry2.pose.pose.position.x = transformOut[3] ;
        laserOdometry2.pose.pose.position.y = transformOut[4] ;
        laserOdometry2.pose.pose.position.z = transformOut[5];
        pubLaserOdometry2.publish(laserOdometry2);
    }

    void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped)
    {
        double roll, pitch, yaw;
        tf::Quaternion quat;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::quaternionMsgToTF(geoQuat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        transformAftMapped[0] = roll;
        transformAftMapped[1] = pitch;
        transformAftMapped[2] = yaw;

        q_temp.x() = geoQuat.x;
        q_temp.y() = geoQuat.y;
        q_temp.z() = geoQuat.z;
        q_temp.w() = geoQuat.w;


        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
        flag = true;
    }

    void laserOdometryHandler2(const nav_msgs::Odometry::ConstPtr &laserOdometry)
    {
        double roll, pitch, yaw;
        tf::Quaternion quat;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::quaternionMsgToTF(geoQuat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        transformAftGtasm[0] = roll;
        transformAftGtasm[1] = pitch;
        transformAftGtasm[2] = yaw;

        transformAftGtasm[3] = laserOdometry->pose.pose.position.x;
        transformAftGtasm[4] = laserOdometry->pose.pose.position.y;
        transformAftGtasm[5] = laserOdometry->pose.pose.position.z;
        flagGtasm = true;

 
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Fusion");

    TransformFusion TFusion;

    ROS_INFO("\033[1;32m---->\033[0m Fusion Started.");

    ros::spin();

    return 0;
}
