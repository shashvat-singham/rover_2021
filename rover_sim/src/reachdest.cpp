#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>
#include <tf/tf.h>
#include<cmath>

double dest_x,dest_y,curr_pos_x,curr_pos_y,prev_x,prev_y,roll,pitch,yaw , speed, angle2;
geometry_msgs::Twist ans;

void dest_cb(geometry_msgs::Pose msg)
{
    dest_x=msg.position.x;
    dest_y=msg.position.y;
}

void curr_pos_cb(nav_msgs::Odometry msg)
{
    curr_pos_x= msg.pose.pose.position.x;
    curr_pos_y= msg.pose.pose.position.y;
    tf::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w

    );
    tf::Matrix3x3 m(q);
    // double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

}

int main(int argc , char** argv)
{
    ros::init(argc,argv,"reachdest");
    ros::NodeHandle nh;

    ros::Subscriber sub_dest = nh.subscribe("/rover/cmd_pose",10,dest_cb);
    ros::Subscriber sub_curr_pos = nh.subscribe("/rover/odom",10,curr_pos_cb);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/rover/cmd_vel",10);
    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/debug/yaw",1);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/debug/angle",1);
    
    ros::Rate loopRate(10);
    // int flag=0;

    while(ros::ok)
    {
        ros::spinOnce();
        float dist,angle;
        // if(flag==0){prev_x=curr_pos_x;prev_y=curr_pos_y;flag=1;}
        // initial_x=curr_pos_x;initial_y=curr_pos_y
        // geometry_msgs::Twist ans
        dist = sqrt((dest_x-curr_pos_x)*(dest_x-curr_pos_x)+(dest_y-curr_pos_y)*(dest_y-curr_pos_y));
        // angle = (atan((curr_pos_y-prev_y)/(-(curr_pos_x-prev_x)))-atan(dest_y-curr_pos_y)/((dest_x-curr_pos_x)))*180/(2*acos(0.0));
        angle = (yaw+atan(dest_y-curr_pos_y)/(-(dest_x-curr_pos_x)))*180/(2*acos(0.0));
        // if(abs(angle)<2)angle=0;
        // // angle= 90- angle;
        // ans.linear.x= 10*dist;
        // if(curr_pos_x-dest_x<0)
        // {
        // if(yaw<0)
        // angle+=180;
        // else
        // angle-=180;
        // }
        
        // // if(angle<0)angle=angle+360;
        // ans.angular.z=angle;
        
        
        // pub.publish(ans);
        // prev_x=curr_pos_x;
        // prev_y=curr_pos_y;
        if(curr_pos_x-dest_x<0)
        {
        if(yaw<0)
        angle+=180;
        else
        angle-=180;
        }
        std_msgs::Float64 angle1;
        if(dist<0.3)
        {
            speed=0;
            angle=0;
        }
        else 
        {
            if(abs(angle)<3)
            {
                speed=10*dist;
                if(abs(angle)<1)
                angle2=0;
                
                // angle.data= 0;
            }
            else
            {
                speed=0;
                angle2=angle;
                // angle1.data= angle;
            }
        }
        
        ans.linear.x=speed;
        ans.angular.z=angle2;
        // angle1.data=angle2;
        // pub2.publish(angle1);
        // std_msgs::Float64 yaw1;
        // yaw1.data=yaw*180/(2*acos(0.0));
        // pub1.publish(yaw1);
        pub.publish(ans);
        




        loopRate.sleep();
    }

}