#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Float64.h>
#include <tf/tf.h>
#include<cmath>
#include<sensor_msgs/Image.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Header.h>
#include<rover_sim/centre.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<vector>

using namespace cv;
using namespace std;

Mat img1;
int thresh=180;

void imageCb(const sensor_msgs::Image& msg)
{
   cv_bridge::CvImagePtr image_ptr;
    try {
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img1 = image_ptr->image;
    } catch(cv_bridge::Exception& err) {
        ROS_ERROR("cv_bridge exception: %s", err.what());
    }
}

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
    ros::init(argc,argv,"reachflag");
    ros::NodeHandle nh;

    ros::Subscriber sub_dest = nh.subscribe("/rover/cmd_pose",10,dest_cb);
    ros::Subscriber sub_curr_pos = nh.subscribe("/rover/odom",10,curr_pos_cb);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/rover/cmd_vel",10);
    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/debug/yaw",1);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/debug/angle",1);

    ros::Subscriber sub = nh.subscribe("/rover/camera/image_raw", 1, imageCb);
    // ros::Subscriber sub = nh.subscribe("/magnus/camera/image_raw", 1, imageCb);
    ros::Publisher pub_center = nh.advertise<rover_sim::centre>("center_detector_node/centre",1);
    ros::Publisher thresh_pub = nh.advertise<sensor_msgs::Image>("detector/thresh", 1);
    ros::Publisher contour_pub = nh.advertise<sensor_msgs::Image>("detector/contours", 1);
    ros::Publisher proc_pub = nh.advertise<sensor_msgs::Image>("detector/processed", 1);
    // ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("center_detector_node/debug",1);
    
    ros::Rate loopRate(10);
    float sumx=0,sumy=0;
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

        double dist1 = (curr_pos_x + 20)*(curr_pos_x + 20) + (curr_pos_y - 16)*(curr_pos_y - 16);
        
        if(dist1<0.2)
        {
            std_msgs::Int32 a;
            if(img1.empty()){a.data=10;pub1.publish(a);continue;}
            else {
                a.data= 1;
                pub1.publish(a);
            }
            Mat img,thresh_img;
            cvtColor(img1, img, CV_BGR2GRAY);
            // inRange(hsv_image, hsv_min, hsv_max, thresh_img);
            Mat gauss_img,canny_output;
                // GaussianBlur(img,gauss_img, Size( 3 , 3), 0);
            threshold(img,thresh_img, thresh, 255, THRESH_BINARY);
            
            bool debug=true;
            if(debug) 
            {
                sensor_msgs::ImagePtr thresh_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", thresh_img).toImageMsg();
                thresh_pub.publish(thresh_msg);
            }

            
            vector<vector<Point>> contours;
            findContours( thresh_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
            cout<<contours.size();
            Mat final_image=img1;


            double maxarea=0;
                int maxcontindex=0;
                for(int i=0;i<contours.size();i++)
                {
                    double area= contourArea(contours[i]);
                    if(area>maxarea&&contours[i].size()==4)
                    {
                    maxcontindex=i;
                    maxarea=area;
                    }
                }

            if(debug) 
            {
                cv::Mat drawing = cv::Mat::zeros(img1.size(), CV_8UC3);
                if(true) { drawing = img1; }
                cv::drawContours(drawing, contours, maxcontindex, cv::Scalar(0,0,255));
                sensor_msgs::ImagePtr contour_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", drawing).toImageMsg();
                contour_pub.publish(contour_msg);
            }
            
            vector<Point>contour=contours[maxcontindex];
            sumx=0;sumy=0;
            for(int i=0;i<contour.size();i++)
            {
                sumx+=contour[i].x;
                sumy+=contour[i].y;
            }
            sumx/=contour.size();
            sumy/=contour.size();

            rover_sim::centre msg1;
            msg1.x= sumx;
            msg1.y= sumy;
            pub.publish(msg1);

            if(debug)
            {
                cv::Mat final_image = cv::Mat::zeros(img1.size(), CV_8UC3);
                if(true) { 
                    final_image = img1; 
                    cv::drawContours(final_image, contours, maxcontindex, cv::Scalar(0,0,255));
                }
                cv::circle(final_image, cv::Point(sumx, sumy), 3, cv::Scalar(0,255,0));
                sensor_msgs::ImagePtr proc_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_image).toImageMsg();
                proc_pub.publish(proc_msg);
            }

        }

        loopRate.sleep();
    }

}