#include <ros/ros.h>
#include <hrl_rfid/RFIDread.h>
#include <hrl_rfid/StringArray_None.h>
#include <vector>
#include <geometry_msgs/Twist.h>

using namespace hrl_rfid;
using namespace std;

bool stillInitializing = true;
int leftEarStrength, rightEarStrength;

void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
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

    ros::Publisher cmdVelpublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    vector<double> desiredZ;
    double speed = 0.1;
    size_t avgSize = 3;

    while(n.ok()){
        if (stillInitializing)
            queryClient.call(modeReq);
            
        desiredZ.push_back(0.07*(double)(leftEarStrength-rightEarStrength));
        if (desiredZ.size() == avgSize+1) {
            desiredZ[0] = desiredZ[avgSize];
            desiredZ.pop_back();
        }
        double average = 0.0;
        for (size_t i = 0; i < desiredZ.size(); i++)
            average += desiredZ[i];
        average /= (double)desiredZ.size();
        
        geometry_msgs::Twist cmdVelMsg;
        cmdVelMsg.angular.z = average;
        cmdVelMsg.angular.x = 0.0;
        cmdVelMsg.angular.y = 0.0;
        cmdVelMsg.linear.x = speed;
        cmdVelpublisher.publish(cmdVelMsg);
        
        ros::spinOnce();    
        loop_rate.sleep();
    }

    return 0;
}
