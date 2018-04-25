#include <cstdlib>

#include <ros/ros.h>

//Mavros files
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/Waypoint.h>


int main(int argc, char **argv){

    // Add subscriber for velocity commands published by detect.cpp
    int rate = 10;

    ros::init(argc, argv, "detect_avoid");
    ros::NodeHandle p;
    ros::NodeHandle n;
    ros::NodeHandle l;

    ros::Rate r(rate);
    
    ////////////////////////////////////////////
    /////////////////Waypoints//////////////////
    ////////////////////////////////////////////
    ros::ServiceClient wp_clear_client = p.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear"); //("waypoint_clear_client")
    ros::ServiceClient wp_srv_client = n.serviceClient<mavros_msgs::WaypointPush>("waypoint_push_client");
    ros::ServiceClient set_home_client = l.serviceClient<mavros_msgs::CommandHome>("/mavros/mission/home"); //("command_home_client")

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

    wp_clear_srv.request = {};

    wp_push_srv.request.start_index = 0;
    wp_push_srv.request.waypoints.push_back(wp_msg);

   set_home_srv.request.current_gps = false;
   set_home_srv.request.latitude = 11;
   set_home_srv.request.longitude = 12;
   set_home_srv.request.altitude = 1;

   if (set_home_client.call(set_home_srv)){
     ROS_INFO("Home was set to new value ");
    }
   else{
     ROS_ERROR("Home position couldn't been changed");
   }


   if (wp_clear_client.call(wp_clear_srv)){
     ROS_INFO("Waypoint list was cleared");
     }
   else{
     ROS_ERROR("Waypoint list couldn't been cleared");
   }

   if (wp_srv_client.call(wp_push_srv)){
     ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
   }
   else{
     ROS_ERROR("Waypoint couldn't been sent");
     ROS_INFO("Success:%d", (bool)wp_push_srv.response.success);
   }

    ////////////////////////////////////////////
    /////////////////GUIDED/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";

    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.success);
    }
    else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed Arming - Check FCU pre-arms (again)");
    }

    ////////////////////////////////////////////
    /////////////////TAKEOFF////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 4; //adjust if height is too low TODO: check THR_MAX value in MP
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }
    
    
    ROS_INFO("Check");

    ////////////////////////////////////////////
    ///////////////////LAND/////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 0;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }

    while (n.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

