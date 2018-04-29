#include <ros/ros.h>
#include <cstdlib>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/TwistStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

// Waypoints
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/Waypoint.h>

#define RAD2DEG(x) ((x)*180./M_PI)

class ScanAndDetect {

public:
    ScanAndDetect(){

        // Initialize Velocity command service
        velocity = n.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 100);

        // Initialize Service Clients
        TOL = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
        cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
        wp_srv_client = n.serviceClient<mavros_msgs::WaypointPush>("waypoint_push_client");
        set_home_client = n.serviceClient<mavros_msgs::CommandHome>("command_home_client");
        arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");

        // Autonomous Flight control Commands
        /////////////////GUIDED/////////////////////

        mavros_msgs::SetMode srv_setMode;
        srv_setMode.request.base_mode = 0;
        srv_setMode.request.custom_mode = "GUIDED";

        if(cl.call(srv_setMode)){
            ROS_INFO("setmode send ok %d value:", srv_setMode.response.success); }
        else{
            ROS_WARN("Failed SetMode"); }

        sleep(2);

        /////////////////Waypoints//////////////////

        mavros_msgs::WaypointPush wp_push_srv;
        mavros_msgs::WaypointClear wp_clear_srv;
        mavros_msgs::CommandHome set_home_srv;

        mavros_msgs::Waypoint wp_msg;

        wp_msg.frame = 0; // mavros_msgs::Waypoint::FRAME_GLOBAL;
        wp_msg.command = 16;
        wp_msg.is_current = false;
        wp_msg.autocontinue = false;
        wp_msg.param1 = 0;
        wp_msg.param2 = 0;
        wp_msg.param3 = 0;
        wp_msg.param4 = 0;
        wp_msg.x_lat = 40.0;
        wp_msg.y_long = 30.0;
        wp_msg.z_alt = 5.0;

        wp_push_srv.request.waypoints.push_back(wp_msg);

        set_home_srv.request.current_gps = false;
        set_home_srv.request.latitude = 11;
        set_home_srv.request.longitude = 12;
        set_home_srv.request.altitude = 1;

        if (set_home_client.call(set_home_srv)){
          ROS_INFO("Home was set to new value "); }
        else{
          ROS_WARN("Home position couldn't been changed"); }

        if (wp_clear_client.call(wp_clear_srv)){
          ROS_INFO("Waypoint list was cleared"); }
        else{
          ROS_WARN("Waypoint list couldn't been cleared"); }

        if (wp_srv_client.call(wp_push_srv)){
          ROS_INFO("Success:%d", (bool)wp_push_srv.response.success); }
        else{
          ROS_WARN("Waypoint couldn't been sent"); }

        sleep(2);

        ///////////////////ARM//////////////////////

        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if(arming_cl.call(srv)){
            ROS_INFO("ARM status: %d", srv.response.success); }
        else{
            ROS_WARN("Failed Arming - Check FCU pre-arms (again)"); }

        /////////////////TAKEOFF////////////////////

        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = 3; //adjust if height is too low
        srv_takeoff.request.latitude = 0;
        srv_takeoff.request.longitude = 0;
        srv_takeoff.request.min_pitch = 0;
        srv_takeoff.request.yaw = 0;

        if(takeoff_cl.call(srv_takeoff)){
            ROS_INFO("Takeoff Status %d", srv_takeoff.response.success);
                    if (srv_takeoff.response.success == 0) {
                        ros::shutdown(); }// kill the node if a pre-arm check prevents takeoff
        }
        else{
            ROS_WARN("Failed Takeoff"); }

        sleep(7); // Allow copter to takeoff and hover before scanner data begins being processed

    // Initialize Subscriber
    sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 400, &ScanAndDetect::scanCallback, this);

    } //end ScanAndDetect

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    // Initialize Publisher variable: msg
       geometry_msgs::TwistStamped msg;

    // Initialize Land service variable srv_land
       mavros_msgs::CommandTOL srv_land;

    int count = scan->scan_time / scan->time_increment;
    int land = 0; //make sure land is only set once

 // Set up velocity control variables
    double yaw       = 0.0;
    double throttle  = 2.0;

    for(int i = 0; i < count; i++) { //Iterate over all scanned degrees

        float degree = RAD2DEG(scan->angle_increment * i);

          if ((degree < 20) || (degree > 320)) {

              if (scan->ranges[i] < .7){ // .7 meter buffer (extends beyond deathly spin of copter blades)

                      ROS_ERROR(":DANGER object: [%f,%f]", degree, scan->ranges[i]);

                      if (land == 0) {
                          land = 1;
                          srv_land.request.altitude = 0;
                          srv_land.request.latitude = 0;
                          srv_land.request.longitude = 0;
                          srv_land.request.min_pitch = 0;
                          srv_land.request.yaw = 0;
                      if(TOL.call(srv_land)){
                          ROS_INFO("LAND Status: %d", srv_land.response.success);
                          sleep(10); // allow time for a safe landing
                          ros::shutdown(); } //shutdown the node after it lands
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
    ros::ServiceClient cl;
    ros::ServiceClient wp_clear_client;
    ros::ServiceClient wp_srv_client;
    ros::ServiceClient set_home_client;
    ros::ServiceClient arming_cl;
    ros::ServiceClient takeoff_cl;

}; // end of class

int main(int argc, char **argv) {

    ros::init(argc, argv, "detect_avoid");

    ScanAndDetect SADObject;

    int rate = 10;
    ros::Rate r(rate);

    ros::spin();

    return 0;
}
