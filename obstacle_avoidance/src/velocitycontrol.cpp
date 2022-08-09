#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>

std_msgs::Float64 a,b,c;
float omega,sep,speed,angle;

void velocitycb(geometry_msgs::Twist msg){
    speed=msg.linear.x;
    angle=(msg.angular.z)*acos(0.0)/90;
    // ros::NodeHandle nh;
    // ros::Publisher pub_vel_lf    = nh.advertise<std_msgs::Float64>("/rover/corner_lf_wheel_lf_controller/command",10);
    // pub_vel_lf.publish(a);
}

int main(int argc ,char** argv)
{
    ros::init(argc, argv, "velcontroller");

    ros::NodeHandle nh;
    double rate;
    nh.getParam("rate",rate);
    ros::Subscriber sub = nh.subscribe("/rover/cmd_vel",10,velocitycb);
    ros::Publisher pub_corner_lf = nh.advertise<std_msgs::Float64>("/rover/bogie_left_corner_lf_controller/command",10);
    ros::Publisher pub_corner_rf = nh.advertise<std_msgs::Float64>("/rover/bogie_right_corner_rf_controller/command",10);
    ros::Publisher pub_corner_lb = nh.advertise<std_msgs::Float64>("/rover/rocker_left_corner_lb_controller/command",10);
    ros::Publisher pub_corner_rb = nh.advertise<std_msgs::Float64>("/rover/rocker_right_corner_rb_controller/command",10);
    ros::Publisher pub_vel_lf    = nh.advertise<std_msgs::Float64>("/rover/corner_lf_wheel_lf_controller/command",10);
    ros::Publisher pub_vel_lm    = nh.advertise<std_msgs::Float64>("/rover/bogie_left_wheel_lm_controller/command",10);
    ros::Publisher pub_vel_lb    = nh.advertise<std_msgs::Float64>("/rover/corner_lb_wheel_lb_controller/command",10);
    ros::Publisher pub_vel_rb    = nh.advertise<std_msgs::Float64>("/rover/corner_rb_wheel_rb_controller/command",10);
    ros::Publisher pub_vel_rm    = nh.advertise<std_msgs::Float64>("/rover/bogie_right_wheel_rm_controller/command",10);
    ros::Publisher pub_vel_rf    = nh.advertise<std_msgs::Float64>("/rover/corner_rf_wheel_rf_controller/command",10);
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/rover/debug",10);
    // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/debug",10);
    ros::Rate loopRate(rate);

    while(ros::ok)
    {
        ros::spinOnce();
        // std_msgs::Float64 v;
        // v.data=20;
        
        // pub.publish(v);
        sep=0.3;
        omega = 50*angle;
        // b.data=omega*sep/2;
        // pub.publish(b);
        a.data=speed+ omega*sep/2;
        pub_vel_lf.publish(a);
        pub_vel_lm.publish(a);
        pub_vel_lb.publish(a);
        a.data=-(speed- omega*sep/2);
        // a.data = -a.data;
        pub_vel_rb.publish(a);
        pub_vel_rm.publish(a);
        pub_vel_rf.publish(a);

        // pub_corner_lf.publish(b);
        
        // pub_corner_lb.publish(b); // pub_corner_lf.publish(b);
        
        // pub_corner_lb.publish(b);
        // b.data=-b.data;
        // pub_corner_rf.publish(b);
        // pub_corner_rb.publish(b);
        // b.data=-b.data;
        // pub_corner_rf.publish(b);
        // pub_corner_rb.publish(b);
        

        loopRate.sleep();
    }

    return 0;
}




