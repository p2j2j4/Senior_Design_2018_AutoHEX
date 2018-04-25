#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>


#define RAD2DEG(x) ((x)*180./M_PI)

class ScanAndDetect {

public:
    ScanAndDetect(){

        // Initialize Velocity command service
        velocity = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 100);

        // Initialize Subscriber
        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &ScanAndDetect::scanCallback, this);
    }

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //int count = scan->scan_time / scan->time_increment;

    // Set up 3D control Variables
//  double ros_roll      = 0; // Exclude: will change on a case to case basis
//  double ros_pitch     = -0.5;
    double ros_yaw       = 0.0;
    double ros_throttle  = 0.0;

//    n.param<double>("ros_roll", ros_roll, 0.0);
//    n.param<double>("ros_pitch", ros_pitch, 0.0);
//    n.param<double>("ros_yaw", ros_yaw, 0.0);
//    n.param<double>("ros_throttle", ros_throttle,0.0);

    // Objects that are within -115 and 115 degrees (in front of copter)
        for(int i = -115; (-116 < i) && (i < 115); i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            // Initialize Publisher variable: msg
            geometry_msgs::TwistStamped msg;

            if (scan->ranges[i] < .3){ //change to .5 for actual coding
                    msg.twist.linear.x = ros_throttle;
                    msg.twist.linear.y = 0;
                    msg.twist.linear.z = 0;
                    msg.twist.angular.x = -5; //pitch backwards hard
                    msg.twist.angular.y = 0;  //roll
                    msg.twist.angular.z = ros_yaw;   //yaw
                    ROS_ERROR(":DANGER object: [%f,%f]", degree, scan->ranges[i]);
                    velocity.publish(msg); }
         }
// TO DO: figure out what the velocity values represent and implement them for diffferent object distances
//            if (.5 < scan->ranges[i] < 3){ //between .5 and 3 meters
//                    urgency = 2; // object is too close
//                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
//                    detection_pub.publish(urgency); }

//            if (3.25 < scan->ranges[i] < 8){ //between 3.25 and 8 meters
//                    urgency = 1; // object is too close
//                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
//                    detection_pub.publish(urgency); }

//           else { //between 8 meters and infinite distance
//                   urgency = 0; // no objects in front
//                  ROS_INFO(": [%f,%f]", degree, scan->ranges[i]);
//                    detection_pub.publish(urgency); }
}
private:
    ros::NodeHandle n;
    ros::Publisher velocity;
    ros::Subscriber sub;

}; // end of class

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect");

    ScanAndDetect SAPObject; //remove object if failure occurs

    ros::spin();

    return 0;
}
