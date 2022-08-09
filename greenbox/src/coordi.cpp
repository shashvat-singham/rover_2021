#include<ros/ros.h>
#include<sensor_msgs/Image.h>
// #include<geometry_msgs/Point.h>
#include<std_msgs/Int32.h>
#include<std_msgs/Float64.h>
#include<std_msgs/Header.h>
#include<greenbox/centre.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<vector>

//image tranporter kis liye?
using namespace cv;
using namespace std;

Mat img1;

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
ros::init(argc, argv, "centre_detector_node");
ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("magnus/camera/image_raw", 1, imageCb);
ros::Publisher pub = nh.advertise<greenbox::centre>("center_detector_node/centre",1);
// ros::Publisher pub1 = nh.advertise<std_msgs::Int32>("center_detector_node/debug",1);
// ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("center_detector_node/debug1",1);
int h_min=80;
int h_max = 100;

ros::Rate loopRate(10);
cv::Scalar hsv_min(h_min,0,0);
cv::Scalar hsv_max(h_max, 255, 255);

while(ros::ok()){
// geometry_msgs::Point msg;
  // ImageConverter ic;
  ros::spinOnce();
  std_msgs::Int32 a;
  if(img1.empty())continue;
    // {a.data=5;std::cout<<"NO IMAGE";}
//   else{
  
// a.data=10;
// }
// pub1.publish(a);
  Mat hsv_image,thresh_img;
  cvtColor(img1, hsv_image, CV_BGR2HSV);
  inRange(hsv_image, hsv_min, hsv_max, thresh_img);


  cv::Mat smooth_image;
  cv::morphologyEx(thresh_img, smooth_image, CV_MOP_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
//  
  
  vector<vector<Point>> contours;
  findContours( smooth_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
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
  // std_msgs::Float64 b;
  // a.data=maxcontindex;pub1.publish(a);
  // b.data=maxarea;pub2.publish(b);
  // Mat drawing=img1;
  // for(int i=0;i<contours.size();i++)
//   cv::drawContours(drawing, contours, maxcontindex, cv::Scalar(0,0,255));
//    namedWindow( "Display window", WINDOW_AUTOSIZE );
// imshow("Display window" ,drawing);
// waitKey(0);
  float sumx=0,sumy=0;
  vector<Point>contour=contours[maxcontindex];
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

  loopRate.sleep();
}
return 0;
}