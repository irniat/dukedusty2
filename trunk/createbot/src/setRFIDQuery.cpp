#include <ros/ros.h>
#include <hrl_rfid/RFIDread.h>
#include <hrl_rfid/StringArray_None.h>

using namespace hrl_rfid;
using namespace std;

bool stillInitializing = true;

void RFIDCallback(const boost::shared_ptr<const RFIDread>& read) {
    stillInitializing = false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "setRFIDQuery");
    ros::NodeHandle n;
    
    ros::Subscriber rfidSub = n.subscribe("/rfid/Chris_RFID_reader", 100, RFIDCallback);
    ros::Rate loop_rate(5);
    
    ros::ServiceClient queryClient = n.serviceClient<StringArray_None>("/rfid/Chris_RFID_mode");
    StringArray_None modeReq;
    modeReq.request.data.push_back("query");

    while(n.ok()){
        if (stillInitializing)
            queryClient.call(modeReq);
        else
            break;
        ros::spinOnce();    
        loop_rate.sleep();
    }
    return 0;
}
