#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <irobot_create_2_1/SensorPacket.h>
#include <irobot_create_2_1/Brake.h>

using namespace irobot_create_2_1;

const double PI = 3.1415;
const int LOOPRATE = 5;
const int RESET_SECONDS = 20;//Reset distance and angle every 20 seconds

typedef struct odom_info_struct {
    float x, y, angle;
} odom_info;

odom_info pose;
geometry_msgs::Twist twist, lastTwist;
double lastDist;
double angleOffset;
bool cliffSensors;

void roombaStateCallback(const boost::shared_ptr<const SensorPacket>& packet) {
    //Assume that angle and dist are cleared before each callback here
    pose.angle = angleOffset + packet->angle * PI / 180.0;
    double Dist = (double)(packet->distance) / 1000.0;
    double dDist = Dist - lastDist;
    pose.x += dDist*cos(pose.angle);
    pose.y += dDist*sin(pose.angle);
    //Debug output
    ROS_INFO("(angle, x, y) = (%.3lf, %.3lf, %.3lf), dDist = %.3lf", pose.angle*180.0/PI, pose.x, pose.y, dDist);
    lastDist = Dist;
    cliffSensors = packet->cliffLeft | packet->cliffFronLeft | packet->cliffFrontRight | packet->cliffRight;
}

void cmdVelCallback(const boost::shared_ptr<const geometry_msgs::Twist>& t) {
    //ROS_INFO("%.3lf %.3lf", t->linear, t->angular);
    twist.linear = t->linear;
    twist.angular = t->angular;
}

double convSpeedFromMstoMMs(double V) {
    double ret = V*1000.0;
    if (ret > 500)  ret = 500;
    if (ret < -500) ret = -500;
    return ret;
}

bool changedTwist() {
    if (twist.linear.x != lastTwist.linear.x)
        return true;
    if (twist.linear.y != lastTwist.linear.y)
        return true;
    if (twist.angular.z != lastTwist.angular.z)
        return true;
    return false;
}

int main(int argc, char** argv){
    char* baseframe;
    char* childframe;
    if (argc < 3) {
        ROS_ERROR("(%i) usage: controlAndOdometer <base frame> <child frame>\n", argc);
        return -1;    
    }
    else {    
        baseframe = argv[1];
        childframe = argv[2];
    }
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    
    ros::Subscriber pos2DSub = n.subscribe("sensorPacket", 100, roombaStateCallback);
    //ros::Subscriber cmdVelSub = n.subscribe("cmd_vel", 100, cmdVelCallback);
    
    /*ros::ServiceClient circleclient = n.serviceClient<Circle>("circle");
    Circle circle;
    ros::ServiceClient tankclient = n.serviceClient<Tank>("tank");
    Tank tank;*/
    ros::ServiceClient brakeclient = n.serviceClient<Brake>("brake");
    Brake brakeMsg;

    ros::Rate loop_rate(LOOPRATE);
    ros::Time current_time;

    int counter = 0;

    while(n.ok()){
        current_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose.angle);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = baseframe;
        odom_trans.child_frame_id = childframe;

        odom_trans.transform.translation.x = pose.x;
        odom_trans.transform.translation.y = pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = baseframe;

        //set the position
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;


        //ROS_INFO("Odometry: (%.3lf, %.3lf) at %.3lf radians\n", pose.x, pose.y, pose.angle);

        //Publish the speed message
        /*if (changedTwist()) {
            //Only update if the command has changed
            double V = sqrt(twist.linear.x*twist.linear.x + twist.linear.y*twist.linear.y);
            double Omega = twist.angular.z;
            double speed = convSpeedFromMstoMMs(V);//Clipped speed in mm/sec
            bool reset = false;
            if (counter > LOOPRATE*RESET_SECONDS) {
                counter = 0;
                angleOffset = pose.angle;
                lastDist = 0.0;
                reset = true;
                ROS_INFO("Resetting...");
            }
            if (Omega > 0.01) {
                int radius = (int)(speed/(2.0*PI*Omega));
                circle.request.speed = 0;//(int)speed;
                circle.request.radius = 0;//radius;
                circle.request.clear = reset;
                if (changedTwist())
                    circleclient.call(circle);
                ROS_INFO("CIRCLE: speed = %.3lf, radius = %i", speed, radius);
            }
            else {
                tank.request.left = speed;
                tank.request.right = speed;
                tank.request.clear = reset;
                if (changedTwist())
                    tankclient.call(tank);
                ROS_INFO("TANK: speed = %.3lf", speed);
            }
        }*/

        if (cliffSensors) {
            brakeMsg.request.brake = true;
            brakeclient.call(brakeMsg);
        }

        odom.child_frame_id = childframe;
        //set the velocity
        //Note: max speed of robot is 500mm/sec, so scale to 0.5
        odom.twist.twist.linear.x = twist.linear.x;
        odom.twist.twist.linear.y = twist.linear.y;
        odom.twist.twist.angular.z = twist.angular.z;

        //publish the odometry message
        odom_pub.publish(odom);
        
        ros::spinOnce();
        loop_rate.sleep();
        lastTwist = twist;
        counter++;
    }
    
    return 0;
}

