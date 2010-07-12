#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_listener");
    ros::NodeHandle node;
    
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (node.ok()) {
        tf::StampedTransform transform;
        try {
            ros::Time past = ros::Time::now() - ros::Duration(5.0);
            //listener.waitForTransform("/base_link", "/testframe", now - ros::Duration(2.0), ros::Duration(2.0));
            listener.lookupTransform("/base_link", "/testframe", past, transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        //ROS_INFO("%.3lf", transform.getOrigin().x());
        printf("%.3lf ", transform.getOrigin().x());
        rate.sleep();
    }
    return 0;
}
