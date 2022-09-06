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
    ros::Subscriber subOdomAftMapped;

    nav_msgs::Odometry laserOdometry2;
    float transformSum[6];
    float transformMapped[6];
    float transformlastleg[6];
    float transformAftMapped[6];
    float transformlastMap[6];
    float transformOut[6];
    deque<vector<float>> mapStore;
    vector<float> mapTemp;
    bool flag = false;
    const int N = 50;

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

    std_msgs::Header currentHeader;

public:
    TransformFusion()
    {

        pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry>("/integrate_init", 1000);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/leg_odom", 1000, &TransformFusion::laserOdometryHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 10, &TransformFusion::odomAftMappedHandler, this);

        for (int i = 0; i < 6; ++i)
        {
            transformSum[i] = 0;
            transformMapped[i] = 0;
            transformlastleg[i] = 0;
            transformAftMapped[i] = 0;
            transformlastMap[i] = 0;
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
        mapTemp.resize(6, 0);
    }

    void transformAssociate() //每一次都在进行腿部里程计的优化
    {
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(transformlastleg[0], Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(transformlastleg[1], Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(transformlastleg[2], Vector3d::UnitZ()));

        q_leg_last = yawAngle * pitchAngle * rollAngle;
        t_leg_last[0] = transformlastleg[3];
        t_leg_last[1] = transformlastleg[4];
        t_leg_last[2] = transformlastleg[5];

        rollAngle = Eigen::AngleAxisd(transformSum[0], Vector3d::UnitX());
        pitchAngle = Eigen::AngleAxisd(transformSum[1], Vector3d::UnitY());
        yawAngle = Eigen::AngleAxisd(transformSum[2], Vector3d::UnitZ());

        q_leg_cur = yawAngle * pitchAngle * rollAngle;
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

        transformMapped[0] = eulerAngle[2]; // rpy,方法没错
        transformMapped[1] = eulerAngle[1];
        transformMapped[2] = eulerAngle[0];
        // std::cout<<"transformMapped:"<<transformMapped[0]<<" "<<transformMapped[1]<<" "<<transformMapped[2]<<std::endl;

        if (flag == true)
        {
            t_cur[0] = transformAftMapped[3] * 0.8 + transformMapped[3] * 0.2;
            t_cur[1] = transformAftMapped[4] * 0.8 + transformMapped[4] * 0.2;
            t_cur[2] = transformAftMapped[5] * 0.8 + transformMapped[5] * 0.2;

            rollAngle = Eigen::AngleAxisd(transformAftMapped[0], Vector3d::UnitX());
            pitchAngle = Eigen::AngleAxisd(transformAftMapped[1], Vector3d::UnitY());
            yawAngle = Eigen::AngleAxisd(transformAftMapped[2], Vector3d::UnitZ());

            Eigen::Quaterniond q_temp = yawAngle * pitchAngle * rollAngle;

            q_cur.x() = q_leg_lidar.x() * 0.2 + q_temp.x() * 0.8;
            q_cur.y() = q_leg_lidar.y() * 0.2 + q_temp.y() * 0.8;
            q_cur.z() = q_leg_lidar.z() * 0.2 + q_temp.z() * 0.8;
            q_cur.w() = q_leg_lidar.w() * 0.2 + q_temp.w() * 0.8;
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

        for (int i = 0; i < 6; i++)
        {
            transformlastleg[i] = transformSum[i];
            transformlastMap[i] = transformMapped[i];
            mapTemp[i] = transformlastMap[i]; //应该先给定大小
        }
        if (mapStore.size() < N)
        {
            mapStore.push_back(mapTemp);
            vector<float> vtmp = mapStore.back();
            for (int i = 0; i < 6; i++)
            {
                transformOut[i] = vtmp[i];
            }
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                transformOut[i] = 0;
            }
            mapStore.pop_front();
            mapStore.push_back(mapTemp);

            for (int i = 0; i < N - 1; i++)
            {
                vector<float> vtmp = mapStore[i];
                for (int j = 0; j < 3; j++)
                {
                    transformOut[j] += vtmp[j]; //前9个值
                }
            }
            for (int i = 0; i < 3; i++)
            {
                transformOut[i] /= (N - 1);
                transformOut[i] = transformOut[i] * 0.88 + mapTemp[i] * 0.12;
            }
        }
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

        //注意获取位置的顺序
        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;
        // std::cout<<"transformSum:"<<transformSum[3]<<" "<<transformSum[4]<<" "<<transformSum[5]<<std::endl;

        transformAssociate();

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformMapped[0], transformMapped[1], transformMapped[2]);
        // std::cout<<"geoQuat:"<<q_cur.coeffs()<<std::endl;

        laserOdometry2.header.stamp = laserOdometry->header.stamp;
        laserOdometry2.pose.pose.orientation.x = geoQuat.x;
        laserOdometry2.pose.pose.orientation.y = geoQuat.y;
        laserOdometry2.pose.pose.orientation.z = geoQuat.z;
        laserOdometry2.pose.pose.orientation.w = geoQuat.w;
        laserOdometry2.pose.pose.position.x = transformMapped[3];
        laserOdometry2.pose.pose.position.y = transformMapped[4];
        laserOdometry2.pose.pose.position.z = transformMapped[5];
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

        transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
        transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
        transformAftMapped[5] = odomAftMapped->pose.pose.position.z;
        flag = false;

        // std::cout<<"t_cur:"<<t_cur[0]<<" "<<t_cur[1]<<" "<<t_cur[2]<<std::endl;
        // std::cout<<"q_cur:"<<q_cur.coeffs()<<std::endl;
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
