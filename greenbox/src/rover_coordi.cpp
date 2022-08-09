#include<ros/ros.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Header.h>
#include<greenbox/centre.h>
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

int main(int argc, char** argv){
ros::init(argc, argv, "centre_detector_node1");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("/rover/camera/image_raw", 1, imageCb);
// ros::Subscriber sub = nh.subscribe("/magnus/camera/image_raw", 1, imageCb);
ros::Publisher pub = nh.advertise<greenbox::centre>("center_detector_node/centre",1);
ros::Publisher thresh_pub = nh.advertise<sensor_msgs::Image>("detector/thresh", 1);
ros::Publisher contour_pub = nh.advertise<sensor_msgs::Image>("detector/contours", 1);p
ros::Publisher proc_pub = nh.advertise<sensor_msgs::Image>("detector/processed", 1);
ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("center_detector_node/debug",1);
// ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("center_detector_node/debug1",1);
// int h_min=10;
// int h_max = 80;

ros::Rate loopRate(10);
// cv::Scalar hsv_min(h_min,0,0);
// cv::Scalar hsv_max(h_max, 255, 255);
float sumx=0,sumy=0;

while(ros::ok()){
// geometry_msgs::Point msg;
  // ImageConverter ic;
  ros::spinOnce();
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

  greenbox::centre msg1;
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

  loopRate.sleep();
}
return 0;
}
