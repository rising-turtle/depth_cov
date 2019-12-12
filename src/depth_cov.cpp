/*
	
	Dec. 12, 2019, He Zhang, hzhang8@vcu.edu 

	analyze depth's data covariance with different  

*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

using namespace std; 
using namespace cv; 

string base_dir(""); 

void processBagfile(string bagfile); 

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "depth_cov");
  
  ros::NodeHandle nh; 

  ROS_INFO("./depth_cov [bagfile]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 

  processBagfile(bagfile); 

  return 0; 
}

void processBagfile(string bagfile)
{
  std::vector<std::string> topics;
  string dpt_tpc = "/cam0/depth"; 
  topics.push_back(dpt_tpc); 
  
  rosbag::Bag bag; 
  bag.open(bagfile, rosbag::bagmode::Read); 
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // for extract cv::mat 
  cv_bridge::CvImageConstPtr cv_ptrD;

  int cnt = 0; 

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        imshow("dpt_file", cv_ptrD->image); 
        cout<<"depth_cov.cpp: show "<<++cnt<<" depth data!"<<endl;
        waitKey(20); 
      }
      if(!ros::ok())
      	break; 
     
  }
  return ; 
}

