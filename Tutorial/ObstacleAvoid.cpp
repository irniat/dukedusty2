#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#define CRUISING_SPEED 0.05

using namespace sensor_msgs;

double turnRate;
double minDist;
double speed;

void laserCallback(const boost::shared_ptr<const LaserScan>& scan) {
    double MAX_ANGLE = scan->angle_max;
    double MIN_ANGLE = scan->angle_min;
    double inc = scan->angle_increment;
    int bins = (int)((MAX_ANGLE - MIN_ANGLE) / inc);
    double minAngle = 0.0;
    double maxAngle = 0.0;
    minDist = scan->range_max;
    double maxDist = 0.0;
    for (int bin = 0; bin < bins; bin++) {
        double thisangle = MIN_ANGLE + bin*inc;
        double range = scan->ranges[bin];
        if (range > scan->range_min && range < minDist) {
            minDist = range;
            minAngle = thisangle;
        }
        if (range < scan->range_max && range > maxDist) {
            maxDist = range;
            maxAngle = thisangle;    
        }
    }
    
    if (minDist > 0.25) {
        //If the minimum distance is big enough, just stay
        //away from potential obstacles
        if (minAngle < 0)
            turnRate = minAngle + 3.141/2.0;
        else
            turnRate = minAngle - 3.141/2.0;
        turnRate = turnRate / 3.141;
        speed = CRUISING_SPEED / (1.0 + 5.0*abs(turnRate));
    }
    else {
        //If the robot has gotten really close to something, stop
        //the robot and turn away towards the area that has 
        //the most open space
        if (maxDist > 1.0)
            turnRate = maxAngle / 3.141;
        else
            turnRate = 0.2;
        speed = 0.0;
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "ObstacleAvoid");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(5);
    ros::Time current_time;
    
    ros::Publisher cmdVelpublisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 50);
    ros::Subscriber laserSub = n.subscribe("scan", 100, laserCallback);

    while(n.ok()){
        current_time = ros::Time::now();

        //ROS_INFO("speed=%.3lf, turnRate=%.3lf", speed, turnRate); 
        
        int automatic;
        n.param("/hallwaydrive/automatic", automatic, 1);
        if (automatic == 0) {
            double paramforward = 0.0, paramturnrate = 0.0;
            n.param("/hallwaydrive/forward", paramforward, 0.05);
            n.param("/hallwaydrive/turnrate", paramturnrate, 0.0);
            turnRate = paramturnrate;
            speed = paramforward;
        }
        
        geometry_msgs::Twist cmdVelMsg;
        cmdVelMsg.angular.z = turnRate;
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

