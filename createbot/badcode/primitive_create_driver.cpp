#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <irobot_create_2_1/Tank.h>
#include <irobot_create_2_1/Circle.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace irobot_create_2_1;

void roombaStateCallback(const boost::shared_ptr<const SensorPacket>& packet) {
    ROS_INFO("dist = %i\n", packet->distance);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    message_filters::Subscriber<SensorPacket> roombaSub(n, "sensorPacket", 1);
    roombaSub.registerCallback(roombaStateCallback);
    
    ros::ServiceClient tankclient = n.serviceClient<Tank>("tank");
    Tank tank;
    ros::ServiceClient circleclient = n.serviceClient<Circle>("circle");
    Circle circle;
    
    ros::Rate loop_rate(4);
    static tf::TransformBroadcaster br;

    while(ros::ok()) {
        /*circle.request.speed = 50;
        circle.request.radius = 500;
        circle.request.clear = false;
        
        if (circleclient.call(circle)) {
            ROS_INFO("Sent circle command with distance %i\n", 50);
        }
        else {
            ROS_ERROR("Error sending tank command\n");
            return 1;
        }*/
        tank.request.left = 100;
        tank.request.right = 100;
        tank.request.clear = false;
        
        if (tankclient.call(tank)) {
            
        }
        else {
            ROS_INFO("Trying to drive...but error (possibly waiting for Roomba to initialize\n");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;    
}
