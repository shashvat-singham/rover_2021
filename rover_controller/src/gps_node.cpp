#include<ros/ros.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <iostream>
#include<geometry_msgs/Pose.h>
#include<sensor_msgs/NavSatFix.h>
#include<nav_msgs/Odometry.h>

using namespace std;
using namespace GeographicLib;

double dest_lat,dest_lon,curr_lon,curr_lat,curr_pos_x,curr_pos_y;

void gps_cb(sensor_msgs::NavSatFix msg)
{
    dest_lat = msg.latitude;
    dest_lon = msg.longitude;
}
void cb(sensor_msgs::NavSatFix msg)
{
    curr_lat= msg.latitude;
    curr_lon= msg.longitude;
}
void curr_pos_cb(nav_msgs::Odometry msg)
{
    curr_pos_x= msg.pose.pose.position.x;
    curr_pos_y= msg.pose.pose.position.y;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"gps_node");
    ros::NodeHandle nh;
    const Geodesic &geod = Geodesic::WGS84();

    double rate;
    nh.getParam("rate",rate);

    ros::Subscriber sub_cur_rpos_gps = nh.subscribe("/fix",10,cb);
    ros::Subscriber sub_desired_gps = nh.subscribe("/rover/desired_point_gps",10,gps_cb);
    ros::Subscriber sub_curr_pos = nh.subscribe("/rover/odom",10,curr_pos_cb);
    ros::Publisher pub_dest = nh.advertise<geometry_msgs::Pose>("/rover/cmd_pose",10);
    ros::Rate loopRate(rate);
    while(ros::ok)
    {
    // dest_x=msg.position.x;
    // dest_y=msg.position.y;
        ros::spinOnce();
        double x1,x2,y1,y2,x_final,y_final;
        bool northp;
        int zone;
        UTMUPS::Forward(dest_lat, dest_lon, zone, northp, x2, y2);
        UTMUPS::Forward(curr_lat, curr_lon, zone, northp, x1, y1);
        x_final = x2-x1+curr_pos_x;
        y_final = y2-y1+curr_pos_y;
        geometry_msgs::Pose ans;
        ans.position.x= x_final;
        ans.position.y= y_final;
        pub_dest.publish(ans);

        loopRate.sleep();
    }

    
    return 0;

}