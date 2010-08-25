#include <ros/ros.h>
#include <hrl_rfid/RFIDread.h>
#include <hrl_rfid/StringArray_None.h>
#include <iostream>
#include <string.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>


using namespace hrl_rfid;
using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;
typedef unsigned char u8;

bool stillInitializing = true;
int leftEarStrength, rightEarStrength;

string getHexString(string hex) {
    string hexConv = "0123456789ABCDEF";
    string ret;
    for (size_t i = 0; i < hex.size(); i++) {
        u8 hi, lo;
        lo = ((u8)hex[i]) & 0x0F;
        hi = (((u8)hex[i]) & 0xF0) >> 4;
        ret += hexConv.substr(hi, 1) + hexConv.substr(lo, 1);
    }
    return ret;
}

void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
    tf::TransformListener listener;
    tf::StampedTransform transform;
    const char* antennaFrame = (read->antenna_name).c_str();
    //listener.waitForTransform("/map", antennaFrame, ros::Time(), ros::Duration(1.0));
    try {
        ros::Time now = ros::Time::now();
        listener.lookupTransform("/map", antennaFrame, now, transform);
        ROS_INFO("%.3f %.3f %.3f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "TestTFRFID");
    ros::NodeHandle n;
    
    ros::Subscriber rfidSub = n.subscribe("/rfid/Chris_RFID_reader", 100, RFIDCallback);
    ros::Rate loop_rate(5);
    

    while(n.ok()){
        
        ros::spinOnce();    
        loop_rate.sleep();
    }

    return 0;
}
