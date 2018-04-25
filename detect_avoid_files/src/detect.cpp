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

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    int count = scan->scan_time / scan->time_increment;
 // float start_ang = scan->angle_min;

    // Set up 3D control Variables
    double yaw       = 0.0;
    double throttle  = 2.0;

    // Objects that are within -115 and 115 degrees (in front of copter)
    for(int i = 0; count; i++) {

     // Initialize Publisher variable: msg
        geometry_msgs::TwistStamped msg;

     float degree = RAD2DEG(scan->angle_increment * i);

          if ((degree > 115) && (degree < 245)) {

              if (scan->ranges[i] < .5){ // .5 meter buffer
                      msg.twist.linear.x = throttle;
                      msg.twist.linear.y = 0;
                      msg.twist.linear.z = 0;
                      msg.twist.angular.x = -5;  //pitch backwards hard
                      msg.twist.angular.y = 0;   //roll
                      msg.twist.angular.z = yaw; //yaw
                      ROS_ERROR(":DANGER object: [%f,%f]", degree, scan->ranges[i]);
                      velocity.publish(msg); }

              if (scan->ranges[i] < 2.5){ // 2.5 meter buffer
                      msg.twist.linear.x = throttle;
                      msg.twist.linear.y = 0;
                      msg.twist.linear.z = 0;
                      msg.twist.angular.x = 0;   //pitch
                      msg.twist.angular.y = 5;   //roll
                      msg.twist.angular.z = yaw; //yaw
                      ROS_WARN(":Object at: [%f,%f] is nearing", degree, scan->ranges[i]);
                      velocity.publish(msg); }

              if (scan->ranges[i] < 5){ //5 meter buffer
                      msg.twist.linear.x = throttle -1; // slow down
                      msg.twist.linear.y = 0;
                      msg.twist.linear.z = 0;
                      msg.twist.angular.x = 0;   //pitch backwards hard
                      msg.twist.angular.y = 0;   //roll kinda hard
                      msg.twist.angular.z = yaw; //yaw
                      ROS_INFO(":Object in range at: [%f] degrees at a distance of [%f]", degree, scan->ranges[i]);
                      velocity.publish(msg); }

            } // end range test between 115 and 245 degrees
         } // end scan itteration
} // end laser scanner callback function

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
