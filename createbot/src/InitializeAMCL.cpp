#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace geometry_msgs;

int main(int argc, char** argv){
    ros::init(argc, argv, "AMCLInitializer");
    ros::NodeHandle n;

    PoseWithCovarianceStamped P;
    P.pose.pose.position.x = 0;
    P.pose.pose.position.y = 0;
    P.pose.pose.position.z = 0;
    P.pose.pose.orientation.x = 0;
    P.pose.pose.orientation.y = 0;
    P.pose.pose.orientation.z = 0;
    P.pose.pose.orientation.w = 1;
    
    ros::Publisher InitialPosePublisher = n.advertise<PoseWithCovarianceStamped>("initialpose", 50);
    ros::Rate loop_rate(5);

    int counter = 0;
    while(n.ok()) {
        InitialPosePublisher.publish(P);
        counter++;
        loop_rate.sleep();
        if (counter > 10)
            break;
    }

    return 0;
}
