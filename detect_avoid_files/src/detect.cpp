#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


#define RAD2DEG(x) ((x)*180./M_PI)

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;

        for(int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);

            // Print all angles and distances to the terminal window
            // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);

            //Show errors when object gets too close to laser scanner
            if (scan->ranges[i] < .3){
                    ROS_ERROR(": [%f,%f]", degree, scan->ranges[i]);
                    ROS_INFO_STREAM(": count: "<< i);
             }
       }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin();

    return 0;
}
