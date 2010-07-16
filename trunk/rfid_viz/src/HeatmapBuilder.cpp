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
#include <map>

using namespace hrl_rfid;
using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

typedef unsigned char u8;

bool stillInitializingRFID = true;
map<string, PointCloud> heatmaps;
map<string, PoseArray> heatmapPoses;
map<string, bool> heatmapSelected;
PointCloud selectedPointCloud;

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

//Use the HSL color model to get a shade of orange,
//then convert it to RGB
double getRGBComponent(double p, double q, double tc) {
    if (tc < 1.0 / 6.0)
            return p + ((q-p)*6*tc);
    if (tc < 0.5)
            return q;
    if (tc < 2.0 / 3.0)
            return p + ((q-p)*6*(2.0/3.0 - tc));
    return p;
}

int getColorHSL(double paramh, double params, double l, int strength) {
    double h = paramh / 256.0;
    double s = params / 256.0;
    double p, q;
    if (l < 0.5)    
            q = l * (l + s);
    else                    
            q = l + s - (l*s);
    p = 2*l - q;
    double tr = h + 1.0 / 3.0;
    double tg = h;
    double tb = h - 1.0 / 3.0;
    if (tr < 0)     tr = tr + 1;if (tr > 1) tr = tr - 1;
    if (tg < 0)     tg = tg + 1;if (tg > 1) tg = tg - 1;
    if (tb < 0)     tb = tb + 1;if (tb > 1) tb = tb - 1;
    int r = (int)(255.0 * getRGBComponent(p, q, tr));
    int g = (int)(255.0 * getRGBComponent(p, q, tg));
    int b = (int)(255.0 * getRGBComponent(p, q, tb));
    int rgb = (((r & 0xff) << 16) | ((g & 0xff) << 8) | (b & 0xff));
    return rgb;
}


void addRead(string hexID, int rssi, tf::StampedTransform trans) {
    if (rssi == -1)  //The tags sometimes see each other; don't record this
        return;
    //Check to see if this tag has been seen yet
    if (heatmaps.find(hexID) == heatmaps.end()) {
        heatmaps[hexID] = PointCloud();
        heatmapPoses[hexID] = PoseArray();
        ChannelFloat32 rgbChannel;
        rgbChannel.name = "rgb";
        heatmaps[hexID].channels.push_back(rgbChannel);
        heatmaps[hexID].header.frame_id = "/map";
        heatmapSelected[hexID] = true;
    }
    double x = trans.getOrigin().x();
    double y = trans.getOrigin().y();
    double z = trans.getOrigin().z();
    Point32 P32;
    P32.x = x;P32.y = y;P32.z = z;
    Point P;
    P.x = x;P.y = y;P.z = z;
    btQuaternion rotation = trans.getRotation();
    Quaternion Q;
    Q.x = rotation.x();Q.y = rotation.y();Q.z = rotation.z();Q.w = rotation.w();
    Pose pose;
    pose.position = P;
    pose.orientation = Q;
    float float_rgb;
    if (rssi > 10) {
        double l = ((double)rssi + 50.0) / 200.0;
        int rgb = getColorHSL(20.0, 240.0, l, rssi);
        float_rgb = *reinterpret_cast<float*>(&rgb);
    }
    heatmaps[hexID].points.push_back(P32);
    //Color of point in RGB channel
    heatmaps[hexID].channels[0].values.push_back(float_rgb);
    heatmapPoses[hexID].poses.push_back(pose);
}

void updateSelectedPointCloud() {
    if (selectedPointCloud.channels.size() == 0) {
        ChannelFloat32 rgbChannel;
        rgbChannel.name = "rgb";
        selectedPointCloud.channels.push_back(rgbChannel);
        selectedPointCloud.header.frame_id = "/map";
    }
    selectedPointCloud.points.clear();
    selectedPointCloud.channels[0].values.clear();
    map<string, bool>::iterator iter = heatmapSelected.begin();
    
    /*
    //Precompute the size of the heatmap based on which tags are selected
    size_t finalSize = 0;
    while (iter != heatmapSelected.end()) {
        string hexID = iter->first;
        if (heatmapSelected[hexID])
            finalSize += heatmaps[hexID].points.size();
        iter++;
    }
    //Allocate space for the merged point cloud
    ret.points = vector<Point32>(finalSize);
    ret.channels[0].values = vector<float>(finalSize);*/

    //Now copy all of the points into the new PointCloud
    iter = heatmapSelected.begin();
    std::back_insert_iterator<vector<Point32> > pbackInsert(selectedPointCloud.points);
    std::back_insert_iterator<vector<float> > cbackInsert(selectedPointCloud.channels[0].values);
    while (iter != heatmapSelected.end()) {
        string hexID = iter->first;
        copy(heatmaps[hexID].points.begin(), heatmaps[hexID].points.end(), pbackInsert);
        copy(heatmaps[hexID].channels[0].values.begin(), 
             heatmaps[hexID].channels[0].values.end(), cbackInsert);
        iter++;
    }
    ROS_INFO("%i", selectedPointCloud.points.size());
}

//The callback function for every time a tag is seen
void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
    /*string antenna_name
    string tagID
    int32 rssi*/
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        ros::Time now = ros::Time::now();
        //listener.lookupTransform("/map", (read->antenna_name).c_str(), now, transform);
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
    }
    string hexID = getHexString(read->tagID);
    addRead(hexID, read->rssi, transform);
    stillInitializingRFID = false;
}

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
    ros::Rate loop_rate(5);
    
    ros::ServiceClient queryClient = n.serviceClient<StringArray_None>(string(RFIDTopicName + "_mode").c_str());
    StringArray_None modeReq;
    modeReq.request.data.push_back("query");

    ros::Publisher heatmapPublisher = n.advertise<PointCloud>("RFIDHeatmapPoints", 50);
    ros::Publisher heatmapPosesPublisher = n.advertise<PoseArray>("RFIDHeatmapPoses", 50);

    int counter = 0;

    while(n.ok()){
        if (stillInitializingRFID)
            queryClient.call(modeReq);
        
        if (counter > 25) {
            counter = 0;
            updateSelectedPointCloud();
            heatmapPublisher.publish(selectedPointCloud);
        }
        
        ros::spinOnce();    
        loop_rate.sleep();
        counter++;
    }

    return 0;
}
