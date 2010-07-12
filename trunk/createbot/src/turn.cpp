#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <irobot_create_rustic/Position2D.h>
#include <irobot_create_rustic/Speeds.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

using namespace irobot_create_rustic;
using namespace sensor_msgs;

int main(int argc, char** argv){
    double turnRate = 0.0;
    double speed = 0.0;
    bool cmdVel = false;
    if (argc >= 3) {
        turnRate = atof(argv[1]);
        speed = atof(argv[2]);
        if (argc > 3)
            cmdVel = (bool)atoi(argv[3]);
    }
    ros::init(argc, argv, "turn");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);
    ros::Time current_time;
    
    ros::Publisher speedController = n.advertise<Speeds>("speeds_bus", 50);
    ros::Publisher cmdVelpublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);

    while(n.ok()){
        current_time = ros::Time::now();
        if (cmdVel) {
            geometry_msgs::Twist cmdVelMsg;
            cmdVelMsg.angular.z = turnRate;
            cmdVelMsg.angular.x = 0.0;
            cmdVelMsg.angular.y = 0.0;
            cmdVelMsg.linear.x = speed;//It doesn't matter what x and y are
            //because my program just takes the absolute magnitude
            cmdVelpublisher.publish(cmdVelMsg);
        }
        else {
            Speeds speedMsg;
            speedMsg.forward = speed;
            speedMsg.rotate = turnRate;
            speedController.publish(speedMsg);
        }
        loop_rate.sleep();
    }
    
    return 0;
}

