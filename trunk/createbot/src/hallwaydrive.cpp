#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <irobot_create_rustic/Position2D.h>
#include <irobot_create_rustic/Speeds.h>
#include <sensor_msgs/LaserScan.h>

using namespace irobot_create_rustic;
using namespace sensor_msgs;

typedef struct odom_info_struct {
    float x, y, angle;
} odom_info;

class Point {
public:
        Point();
        Point(double px, double py);
        double x, y;
        //Assignment operator
        Point& operator=(const Point& point);
        Point& operator+=(const Point& point);
        Point& operator*=(const double a);

        double getAngle();
        double getSquaredMag();
        double getMag();
        void print();
};

Point::Point() {
        x = 0.0; y = 0.0;
}

Point::Point(double px, double py) {
        x = px; y = py;
}

Point& Point::operator=(const Point& point) {
        x = point.x;y=point.y;
        return *this;
}

Point& Point::operator+=(const Point& point) {
        x += point.x;y += point.y;
        return *this;
}

Point& Point::operator*=(const double a) {
        x = x * a; y = y * a;
        return *this;
}

double Point::getAngle() {
        //Rotate the angle by 90 degrees, so that zero degrees occurs when x = 0
        //a negative angle occurs when x < 0, and a positive angle occurs when x > 0
        //Basically, we want to figure out the angle that the point makes with the forward direction
        return atan(-x / y);
}

double Point::getSquaredMag() {
        return x*x + y*y;
}

double Point::getMag() {
        return sqrt(x*x + y*y);
}

void Point::print() {
        printf("(%f, %f)\n", x, y);
}


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////


class OccupancyGrid {
public:
        double increment, owidth, oheight;//in meters
        int oarrwidth, oarrheight;//Width and heigh of occupancy grid (in increment units)      
        bool** grid;
        double closeObstacleAngle;
        double minDist;
        bool closeObstacle;
        
        OccupancyGrid(double inc, double width, double height);
        ~OccupancyGrid();
        void fill(const boost::shared_ptr<const LaserScan>& scan);
        void output();
        Point GetCoord(int x, int y);
        Point GetCentroid();
        Point GetRightBiasCentroid(double centerX);
        double getSpeedFactor();
        void findCloseObstacleBin(const boost::shared_ptr<const LaserScan>& scan);
        double getObstacleLaserAngle();
};

OccupancyGrid::OccupancyGrid(double inc, double width, double height) {
        increment = inc;
        owidth = width;
        oheight = height;
        oarrwidth = (int)(owidth / increment);
        oarrheight = (int)(oheight / increment);
        grid = (bool**)calloc(oarrwidth, sizeof(bool**));
        for (int x = 0; x < oarrwidth; x++) {
                grid[x] = (bool*)calloc(oarrheight, sizeof(bool*));
        }
}

OccupancyGrid::~OccupancyGrid() {
        for (int i = 0; i < oarrwidth; i++) {
                free(grid[i]);
        }
        free(grid);
}

void OccupancyGrid::fill(const boost::shared_ptr<const LaserScan>& scan) {
        if (scan == NULL)
            return;
        //look in variable scan
        double MAX_ANGLE = scan->angle_max;
        double MIN_ANGLE = scan->angle_min;
        double inc = scan->angle_increment;
        int bins = (int)((MAX_ANGLE - MIN_ANGLE) / inc);
        for (int x = 0; x < oarrwidth; x++) {
                double dx = increment * (double)(x - oarrwidth / 2);
                for (int y = 0; y < oarrheight; y++) {
                        double dy = increment * (double)y;
                        Point point(dx, dy);
                        double angle = point.getAngle();
                        double sqrdist = point.getSquaredMag();
                        //Find the nearest bin in the laser scan (converting from rectangular to polar)
                        //NOTE: This is the simple approach for now, but could cause aliasing
                        int bin = (int)(angle / inc) + bins / 2;
                        if (scan->ranges[bin]*scan->ranges[bin] > sqrdist)
                                grid[x][y] = false;//The cell is empty
                        else
                                grid[x][y] = true;//The cell is full
                }
        }
}


//Output grid to ASCII file
void OccupancyGrid::output() {
        FILE* file = fopen("grid.txt", "w");
        for (int x = 0; x < oarrwidth; x++) {
                for (int y = 0; y < oarrheight; y++) {
                        if (grid[x][y]) fprintf(file, "* ");//Obstacle in the way
                        else fprintf(file, "- ");//Open space
                }
                fprintf(file, "\n");
        }
        fclose(file);
}

//Convert from array index to position offset in meters
Point OccupancyGrid::GetCoord(int x, int y) {
        Point coord;
        coord.x = (double)(x - oarrwidth / 2) * increment;
        coord.y = (double)y * increment;
        return coord;
}

//Get the centroid of the open area
Point OccupancyGrid::GetCentroid() {
        Point toReturn;
        double total = 0.0;
        for (int x = 0; x < oarrwidth; x++) {
                for (int y = 0; y < oarrheight; y++) {
                        if (!grid[x][y]) {  //If the cell is empty, weight it in the centroid
                                toReturn += GetCoord(x, y);
                                total++;                        
                        }
                }
        }
        toReturn.x /= total; toReturn.y /= total;
        return toReturn;
}

Point OccupancyGrid::GetRightBiasCentroid(double centerX) {
        Point toReturn;
        double total = 0.0;
        for (int x = 0; x < oarrwidth; x++) {
                for (int y = 0; y < oarrheight; y++) {
                        if (!grid[x][y]) {  //If the cell is empty, weight it in the centroid
                                Point coord = GetCoord(x, y);          
                                if (coord.x >= centerX) {                               
                                        toReturn += coord; total ++;
                                }               
                        }
                }
        }
        toReturn.x /= total; toReturn.y /= total;
        return toReturn;
}

double OccupancyGrid::getSpeedFactor() {
    int open = 0, closed = 0;
    for (int x = 0; x < oarrwidth; x++) {
        for (int y = 0; y < oarrheight; y++) {
            if (!grid[x][y]) open++;
            else    closed++;
        }
    }
    return (double)open / (double)(closed + 10);
}

void OccupancyGrid::findCloseObstacleBin(const boost::shared_ptr<const LaserScan>& scan) {
    if (scan == NULL)
        return;
    double MAX_ANGLE = scan->angle_max;
    double MIN_ANGLE = scan->angle_min;
    double inc = scan->angle_increment;
    int bins = (int)((MAX_ANGLE - MIN_ANGLE) / inc);
    double angle = 0.0;
    minDist = scan->range_max;
    for (int bin = 0; bin < bins; bin++) {
        double thisangle = MIN_ANGLE + bin*inc;
        if (abs(thisangle) > 3.141/3.0)
            continue;
        if (scan->ranges[bin] < minDist && scan->ranges[bin] > scan->range_min) {
            minDist = scan->ranges[bin];
            angle = thisangle;
        }
    }
    if (minDist < 0.5) {
        closeObstacleAngle = angle;
        closeObstacle = true;
    }
    else
        closeObstacle = false;
}

double OccupancyGrid::getObstacleLaserAngle() {
    return closeObstacleAngle;
}

odom_info pose;
OccupancyGrid grid(0.1, 4.0, 2.0);

void roombaOdomCallback(const boost::shared_ptr<const Position2D>& position) {
    pose.x = position->x;
    pose.y = position->y;
    pose.angle = position->a;
}

void laserCallback(const boost::shared_ptr<const LaserScan>& s) {
    grid.fill(s);
    grid.findCloseObstacleBin(s);
}


int main(int argc, char** argv){
    double cruising_speed = 0.1;
    char* baseframe;
    char* childframe;
    if (argc < 4) {
        ROS_ERROR("(%i) usage: hallwaydrive <speed> <base frame> <child frame>\n", argc);
        return -1;    
    }
    else {    
        cruising_speed = atof(argv[1]);
        baseframe = argv[2];
        childframe = argv[3];
    }
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate loop_rate(5);
    ros::Time current_time;
    
    ros::Publisher speedController = n.advertise<Speeds>("speeds_bus", 50);
    
    /*message_filters::Subscriber<Position2D> pos2DSub(n, "/pos2d_bus", 1);
    pos2DSub.registerCallback(roombaOdomCallback);
    
    message_filters::Subscriber<LaserScan> laserSub(n, "/scan", 1);
    laserSub.registerCallback(laserCallback);*/
    
    ros::Subscriber pos2DSub = n.subscribe("pos2d_bus", 100, roombaOdomCallback);
    ros::Subscriber laserSub = n.subscribe("scan", 100, laserCallback);

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

        double speed = 0.0, angle = 0.0;
        /*if (grid.closeObstacle) {
            double closestAngle = grid.getObstacleLaserAngle();
            ROS_INFO("Close Obstacle at angle %.3lf at dist %lf", closestAngle, grid.minDist);
            //If there's an obstacle really close, turn away from that obstacle
            if (closestAngle < 0)
                angle = closestAngle + 3.141/2.0;
            else
                angle = closestAngle - 3.141/2.0;
            angle /= (1.0 + grid.minDist*4.0);
        }
        else {*/
            Point centroid = grid.GetCentroid();
            angle = centroid.getAngle();
            angle = angle / 3.141;
            angle = angle*angle*angle;
            angle *= 3.141;
        //}
        speed = cruising_speed / (1.0 + 5.0 * fabs(angle));

        //Publish the speed message
        Speeds speedMsg;
        speedMsg.forward = speed;
        speedMsg.rotate = angle;
        
        if(!n.hasParam("/hallwaydrive/automatic")) {
            ROS_ERROR("Unable to find parameter automatic");
            return -1;
        }
        
        int automatic;
        n.param("/hallwaydrive/automatic", automatic, 0);
        //ROS_INFO("automatic = (%i)", automatic);
        if (automatic == 1) { //Only send commands if the user wants
        //automatic control; otherwise they will interfere with the controller GUI
            speedController.publish(speedMsg);
        }
        else if (automatic == 0) {
            double forward = 0.0, turnrate = 0.0;
            n.param("/hallwaydrive/forward", forward, 0.05);
            n.param("/hallwaydrive/turnrate", turnrate, 0.0);
            speedMsg.forward = forward;
            speedMsg.rotate = turnrate;
            speedController.publish(speedMsg);
        }

        //ROS_INFO("speed = %.3f, turn = %.3f\n", speed, angle);


        odom.child_frame_id = childframe;
        //set the velocity
        //Note: max speed of robot is 500mm/sec, so scale to 0.5
        odom.twist.twist.linear.x = speed*0.5*cos(pose.angle);
        odom.twist.twist.linear.y = speed*0.5*sin(pose.angle);
        odom.twist.twist.angular.z = angle;

        //publish the odometry message
        odom_pub.publish(odom);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

