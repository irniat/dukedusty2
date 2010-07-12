#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

static std::string hokuyo_name;

int main(int argc, char** argv){
    if (argc < 2) {
        ROS_ERROR("Need to specify name for hokuyo");
        return -1;
    }

    hokuyo_name = argv[1];

    ros::init(argc, argv, "hokuyo_tf_broadcaster");

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0, 0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0));

    ros::Rate loop_rate(10);
    static tf::TransformBroadcaster br;

    while(ros::ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", hokuyo_name));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_camera", "usb_cam"));
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
