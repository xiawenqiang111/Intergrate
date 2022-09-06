#include "ros/ros.h"
#include "tf/transform_datatypes.h"//转换函数头文件
#include <nav_msgs/Odometry.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;
class FusionGtasm
{
public:
 
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values isamCurrentEstimate;
    ISAM2 *isam;
    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    ros::NodeHandle nh;

    ros::Publisher pubLaserOdometry;
    ros::Subscriber subLaserOdometry;

	nav_msgs::Odometry laserOdometry;


    float transformMapped[6];
    float transformLast[6];
    float transformOut[6];
    long long int size = 0;
   
  

public:
    FusionGtasm()
    {
        pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/integrate_init", 1000);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/high_frec_init", 1000, &FusionGtasm::laserOdometryHandler, this);
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 3;
        isam = new ISAM2(parameters);
        gtsam::Vector Vector6(6);
        Vector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
        odometryNoise = noiseModel::Diagonal::Variances(Vector6);
        for(int i=0;i<6;i++)
        {
            transformMapped[i] = 0;
            transformLast[i] = 0;
            transformOut[i] = 0;
        }
    }

    void run()
    {
      
        if (size == 0)
        {
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformLast[0], transformLast[1], transformLast[2]), 
                                            Point3(transformLast[3], transformLast[4], transformLast[5])), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformLast[0], transformLast[1], transformLast[2]),
                                            Point3(transformLast[3], transformLast[4], transformLast[5])));
            for (int i = 0; i < 6; ++i)
                transformLast[i] = transformMapped[i];
        }
        else
        {
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[0], transformLast[1], transformLast[2]),
                                          Point3(transformLast[3], transformLast[4], transformLast[5]));
            gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(transformMapped[0], transformMapped[1], transformMapped[2]),
                                        Point3(transformMapped[3], transformMapped[4], transformMapped[5]));
            gtSAMgraph.add(BetweenFactor<Pose3>(size-1, size , poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(size, Pose3(Rot3::RzRyRx(transformLast[0], transformLast[1], transformLast[2]),
                                                                         Point3(transformMapped[3], transformMapped[4], transformMapped[5])));
        }
  
        /**
         * update iSAM
         */
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        Pose3 latestEstimate;
        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
       
        transformOut[0] = latestEstimate.rotation().roll();
        transformOut[1] = latestEstimate.rotation().pitch();
        transformOut[2] = latestEstimate.rotation().yaw();
        transformOut[3] = latestEstimate.translation().x();
        transformOut[4] = latestEstimate.translation().y();
        transformOut[5] = latestEstimate.translation().z();

        for (int i = 0; i < 6; ++i){
            transformLast[i] = transformMapped[i];
        }
        //  cout<<"处理前的位姿:"<<transformMapped[0]<<","<<transformMapped[1]<<","<<transformMapped[2]<<
        //      ","<<transformMapped[3]<<","<<transformMapped[4]<<","<<transformMapped[5]<<endl;

        //  cout<<"处理后的位姿:"<<transformOut[0]<<","<<transformOut[1]<<","<<transformOut[2]<<
        //      ","<<transformOut[3]<<","<<transformOut[4]<<","<<transformOut[5]<<endl;

    
    
    }
    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomAftMapped)
    {
        
        double roll, pitch, yaw;
        tf::Quaternion quat;
        geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
        tf::quaternionMsgToTF(geoQuat, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        transformMapped[0] = roll;
        transformMapped[1] = pitch;
        transformMapped[2] = yaw;

        transformMapped[3] = odomAftMapped->pose.pose.position.x;
        transformMapped[4] = odomAftMapped->pose.pose.position.y;
        transformMapped[5] = odomAftMapped->pose.pose.position.z;


        run();
       

        size++;


        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                  (transformOut[0], transformOut[1], transformOut[2]);
	

        laserOdometry.header.stamp =  odomAftMapped->header.stamp;
        laserOdometry.pose.pose.orientation.x = geoQuat.x;
        laserOdometry.pose.pose.orientation.y = geoQuat.y;
        laserOdometry.pose.pose.orientation.z = geoQuat.z;
        laserOdometry.pose.pose.orientation.w = geoQuat.w;
        laserOdometry.pose.pose.position.x = transformOut[3];
        laserOdometry.pose.pose.position.y = transformOut[4];
        laserOdometry.pose.pose.position.z = transformOut[5];
        pubLaserOdometry.publish(laserOdometry);

    }
};

int main(int argc, char** argv)
{
   
    ros::init(argc, argv, "Fusion");
    
    FusionGtasm FGtasm;

    ROS_INFO("\033[1;32m---->\033[0m GTSAM Started.");

    ros::spin();

    return 0;
}
