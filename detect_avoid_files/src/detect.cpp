#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vel_cmd/FlightCmd.h>


#define RAD2DEG(x) ((x)*180./M_PI)

class ScanAndDetect {

public:
    ScanAndDetect(){

        // Initialize Publisher
        ros::Pubisher detection_pub = n.advertise<vel_cmd::FlightCmd>("/detection", 100);

        // Initialize Subscriber
        ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &ScanAndDetect::scanCallback, this);
    }

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;

    // Figure out ranges for each for loop
        for(int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            //Publish urgency value (0-5) when object gets too close to laser scanner
            //Urgency from 0-3 is for a scan range in front of the copter
            vel_cmd::FlightCmd urgency;

            if (scan->ranges[i] < .3){ //change to .5 for actual coding
                    urgency = 3; // object is too close
                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
                    detection_pub.publish(urgency); }

            if (.5 < scan->ranges[i] < 3){ //between .5 and 3 meters
                    urgency = 2; // object is too close
                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
                    detection_pub.publish(urgency); }

            if (3.25 < scan->ranges[i] < 8){ //between 3.25 and 8 meters
                    urgency = 1; // object is too close
                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
                    detection_pub.publish(urgency); }

            else { //between 8 meters and infinite distance
                    urgency = 0; // no objects in front
                    ROS_INFO(": [%f,%f]", degree, scan->ranges[i]);
                    detection_pub.publish(urgency); }
       }
}
private:
    ros::NodeHandle n;
    ros::Publisher detection_pub;
    ros::Subscriber sub;

} // end of class

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect");

    ScanAndDetect SAPObject;

    ros::spin();

    return 0;
}
