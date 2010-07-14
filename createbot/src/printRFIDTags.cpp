#include <ros/ros.h>
#include <hrl_rfid/RFIDread.h>
#include <hrl_rfid/StringArray_None.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>

using namespace hrl_rfid;
using namespace std;

/*string antenna_name
string tagID
int32 rssi*/

bool stillInitializing = true;
FILE* leftEar;
FILE* rightEar;
int leftEarStrength, rightEarStrength;

void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
    //ROS_INFO("%s: %s: RSSI = %i", (read->antenna_name).data(), (read->tagID).data(), read->rssi);
    //ROS_INFO("%s (%i)", read->tagID.c_str(), read->rssi);
    //ROS_INFO("%s: RSSI = %i", read->antenna_name.c_str(), read->rssi);
    /*tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        ros::Time now = ros::Time::now();
        listener.lookupTransform("/odom", "/base_link", now, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    //double angle = 2.0*acos(transform.transform.rotation.w)*(180.0/3.14);
    double angle = transform.getRotation().getAngle()*(180.0/3.141);
    if(read->antenna_name.compare("EleLeftEar") == 0) {
        fprintf(leftEar, "%.1f %i ", angle, read->rssi);
    }
    else {
        fprintf(rightEar, "%.1f %i ", angle, read->rssi);
    }
    ROS_INFO("%.1lf %i", angle, read->rssi);*/
    if (read->antenna_name.compare("EleLeftEar") == 0)
        leftEarStrength = read->rssi;
    else
        rightEarStrength = read->rssi;
    
    stillInitializing = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "printRFIDTags");
    ros::NodeHandle n;
    
    ros::Subscriber rfidSub = n.subscribe("/rfid/Chris_RFID_reader", 100, RFIDCallback);
    ros::Rate loop_rate(5);
    
    ros::ServiceClient queryClient = n.serviceClient<StringArray_None>("/rfid/Chris_RFID_mode");
    StringArray_None modeReq;
    modeReq.request.data.push_back("query");

    leftEar = fopen("~/ros/ros/dukedusty2/createbot/leftEar.txt", "w");
    rightEar = fopen("~/ros/ros/dukedusty2/createbot/rightEar.txt", "w");

    ros::Publisher cmdVelpublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    double desiredZ[5];
    double speed = 0.05;

    while(n.ok()){
        if (stillInitializing)
            queryClient.call(modeReq);
            
        geometry_msgs::Twist cmdVelMsg;
        cmdVelMsg.angular.z = 0.07*(double)(leftEarStrength-rightEarStrength);
        cmdVelMsg.angular.x = 0.0;
        cmdVelMsg.angular.y = 0.0;
        cmdVelMsg.linear.x = speed;//It doesn't matter what x and y are
        //because my program just takes the absolute magnitude
        cmdVelpublisher.publish(cmdVelMsg);
        
        ros::spinOnce();    
        loop_rate.sleep();
    }

    return 0;
}
