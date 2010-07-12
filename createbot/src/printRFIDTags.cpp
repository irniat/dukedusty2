#include <ros/ros.h>
#include "/home/createbrain/ros/ros/hrl/hrl_rfid/msg/cpp/hrl_rfid/RFIDread.h"
#include "/home/createbrain/ros/ros/hrl/hrl_rfid/srv/cpp/hrl_rfid/StringArray_None.h"
#include <iostream>

using namespace hrl_rfid;
using namespace std;

/*string antenna_name
string tagID
int32 rssi*/
void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
    //ROS_INFO("%s: %s: RSSI = %i", (read->antenna_name).data(), (read->tagID).data(), read->rssi);
    //ROS_INFO("%s (%i)", read->tagID.c_str(), read->rssi);
    cout << read->tagID << "  " << read->tagID.size() << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "printRFIDTags");
    ros::NodeHandle n;
    
    ros::Subscriber rfidSub = n.subscribe("/rfid/Chris_RFID_reader", 100, RFIDCallback);
    ros::Rate loop_rate(5);
    
    ros::ServiceClient queryClient = n.serviceClient<StringArray_None>("/rfid/Chris_RFID_mode");
    StringArray_None modeReq;
    modeReq.request.data.push_back("query");
    queryClient.call(modeReq);

    while(n.ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
