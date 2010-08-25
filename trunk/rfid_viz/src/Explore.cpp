#include <ros/ros.h>
#include <iostream>
#include <string.h>
#include <map>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace nav_msgs;

int main(int argc, char** argv) {
    //TODO: Make the TF frame IDs into ROS parameters
    ros::init(argc, argv, "HeatmapBuilder");
    ros::NodeHandle n;
    string RFIDTopicName, heatmapOut;

    RFIDTopicName = "/rfid/Chris_RFID";
    
    /*if (argc > 4) {
        RFIDTopicName = "/rfid/" + string(argv[1]);
    }
    else {
        ROS_ERROR("Params needed: <RFID Topic Name> <Bag File>");
    }*/
    
    ros::Subscriber rfidSub = n.subscribe(string(RFIDTopicName + "_reader").c_str(), 100, RFIDCallback);

    
    ros::ServiceClient queryClient = n.serviceClient<StringArray_None>(string(RFIDTopicName + "_mode").c_str());
    StringArray_None modeReq;
    modeReq.request.data.push_back("query");

    ros::Publisher heatmapPublisher = n.advertise<PointCloud>("RFIDHeatmapPoints", 50);
    ros::Publisher heatmapPosesPublisher = n.advertise<PoseArray>("RFIDHeatmapPoses", 50);
    
    while(n.ok()){
        if (stillInitializingRFID)
            queryClient.call(modeReq);
        
        updateSelectedPointCloud();
        if (selectedPointCloud.points.size() > 0) {
            heatmapPublisher.publish(selectedPointCloud);
        }
        
        ros::spinOnce();    
        ros::Duration d = ros::Duration(0.2, 0);
        d.sleep();
    }
    ROS_INFO("Quitting main");

    return 0;
}
