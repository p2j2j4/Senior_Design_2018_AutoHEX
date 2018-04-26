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
        sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 400, &ScanAndDetect::scanCallback, this);

        // Initialize Land Client
        TOL = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    }

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    // Initialize Publisher variable: msg
       geometry_msgs::TwistStamped msg;

    // Initialize Land service variable srv_land
       mavros_msgs::CommandTOL srv_land;

    int count = scan->scan_time / scan->time_increment;

 // Set up velocity control variables
    double yaw       = 0.0;
    double throttle  = 2.0;

    for(int i = 0; i < count; i++) { //Iterate over all scanned degrees

        float degree = RAD2DEG(scan->angle_increment * i);
        bool land = false;

          if ((degree < 20) || (degree > 320)) {

              if (scan->ranges[i] < .5){ // .5 meter buffer

                      ROS_ERROR(":DANGER object: [%f,%f]", degree, scan->ranges[i]);

                      if (!land) {
                          land = true;
                          srv_land.request.altitude = 0;
                          srv_land.request.latitude = 0;
                          srv_land.request.longitude = 0;
                          srv_land.request.min_pitch = 0;
                          srv_land.request.yaw = 0;
                      if(TOL.call(srv_land)){
                          ROS_INFO("LAND Status: %d", srv_land.response.success); }
                      }
              }

              if ((scan->ranges[i] < 2) && (scan->ranges[i] >= .5)){ // 2 meter buffer
                      msg.twist.linear.x = throttle;
                      msg.twist.linear.y = 0;
                      msg.twist.linear.z = 0;
                      msg.twist.angular.x = 0;   //pitch
                      msg.twist.angular.y = 5;   //roll
                      msg.twist.angular.z = yaw; //yaw
                      ROS_WARN(":Object at: [%f,%f] is nearing", degree, scan->ranges[i]);
                      velocity.publish(msg); }

              if ((scan->ranges[i] < 4) && (scan->ranges[i] >= 2)){ //4 meter buffer
                      msg.twist.linear.x = throttle -1; // slow down? TO DO: Learn what throttle values mean
                      msg.twist.linear.y = 0;
                      msg.twist.linear.z = 0;
                      msg.twist.angular.x = 0;   //pitch
                      msg.twist.angular.y = 0;   //roll
                      msg.twist.angular.z = yaw; //yaw
                      ROS_INFO(":Object in range at: [%f] degrees at a distance of [%f]", degree, scan->ranges[i]);
                      velocity.publish(msg); }

            } // end range test between 320 and 20 degrees
          } // end scan itteration
} // end laser scanner callback function

private:
    ros::NodeHandle n;
    ros::Publisher velocity;
    ros::Subscriber sub;
    ros::ServiceClient TOL;

}; // end of class

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect");

    ScanAndDetect SAPObject; //remove object if failure occurs

    ros::spin();

    return 0;
}
