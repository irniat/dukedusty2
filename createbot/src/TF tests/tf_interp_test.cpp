#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <irobot_create_rustic/Position2D.h>
#include <irobot_create_rustic/Speeds.h>
#include <sensor_msgs/LaserScan.h>

using namespace irobot_create_rustic;
using namespace sensor_msgs;

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_test");
    ros::NodeHandle n;
    ros::Time current_time;
    
    tf::TransformBroadcaster tfBroadcaster;
    ros::Rate loop_rate(1);

    double x = 0.1;
    double velocity = 0.1;

    while(n.ok()){
        current_time = ros::Time::now();
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
                
        geometry_msgs::TransformStamped tfMessage;
        tfMessage.header.stamp = current_time;
        tfMessage.header.frame_id = "base_link";
        tfMessage.child_frame_id = "testframe";

        tfMessage.transform.translation.x = x;
        tfMessage.transform.translation.y = 0;
        tfMessage.transform.translation.z = 0.0;
        tfMessage.transform.rotation = odom_quat;
        
        tfBroadcaster.sendTransform(tfMessage);
        
        x*=2.0;
        
        printf("%.3f ", x);
        
        loop_rate.sleep();
    }
    
    return 0;
}

