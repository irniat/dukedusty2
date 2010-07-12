#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <irobot_create_rustic/Position2D.h>
#include <irobot_create_rustic/Speeds.h>
#include <sensor_msgs/LaserScan.h>

#define ANGULAR_CONVERSION_FACTOR 3.485

using namespace irobot_create_rustic;
using namespace sensor_msgs;


typedef struct odom_info_struct {
    float x, y, angle;
} odom_info;

odom_info pose;
geometry_msgs::Twist twist;

void roombaOdomCallback(const boost::shared_ptr<const Position2D>& position) {
    pose.x = position->x;
    pose.y = position->y;
    pose.angle = position->a;
}

void cmdVelCallback(const boost::shared_ptr<const geometry_msgs::Twist>& t) {
    //ROS_INFO("%.3lf %.3lf", t->linear, t->angular);
    twist.linear = t->linear;
    twist.angular = t->angular;
}

int main(int argc, char** argv){
    char* baseframe;
    char* childframe;
    if (argc < 3) {
        ROS_ERROR("(%i) usage: controlAndOdometer <base frame> <child frame>\n", argc);
        return -1;    
    }
    else {    
        baseframe = argv[1];
        childframe = argv[2];
    }
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate loop_rate(5);
    ros::Time current_time;
    
    ros::Publisher speedController = n.advertise<Speeds>("speeds_bus", 50);
    
    ros::Subscriber pos2DSub = n.subscribe("pos2d_bus", 100, roombaOdomCallback);
    ros::Subscriber cmdVelSub = n.subscribe("cmd_vel", 100, cmdVelCallback);

    while(n.ok()){
        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.angle);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = baseframe;
        odom_trans.child_frame_id = childframe;

        odom_trans.transform.translation.x = pose.x;
        odom_trans.transform.translation.y = pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = baseframe;

        //set the position
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;


        //ROS_INFO("Odometry: (%.3lf, %.3lf) at %.3lf radians\n", pose.x, pose.y, pose.angle);

        //Publish the speed message
        double speed = sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
        //ROS_INFO("speed = %.3lf, rotate = %.3lf", speed, twist.angular.z);
        Speeds speedMsg;
        speedMsg.forward = speed;
        speedMsg.rotate = twist.angular.z / ANGULAR_CONVERSION_FACTOR;

        speedController.publish(speedMsg);



        odom.child_frame_id = childframe;
        //set the velocity
        //Note: max speed of robot is 500mm/sec, so scale to 0.5
        odom.twist.twist.linear.x = twist.linear.x;
        odom.twist.twist.linear.y = twist.linear.y;
        odom.twist.twist.angular.z = twist.angular.z;

        //publish the odometry message
        odom_pub.publish(odom);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

