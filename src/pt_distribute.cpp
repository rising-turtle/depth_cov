/*

	Dec.19 2019, He Zhang, hzhang8@vcu.edu

	extract data at a point and analyze its distribution

*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <numeric>
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

bool g_inverse_depth = false;
void processBagfile(string bagfile);
string output_log("output.log");

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pt_distribute");

  ros::NodeHandle nh;

  ROS_INFO("./pt_distribute [bagfile] [output.log] [inv_dep=true]");

  string bagfile = "";
  if(argc >= 2)
    bagfile = argv[1];

  if(argc >= 3)
  	output_log = argv[2];

  if(argc >= 4)
  	g_inverse_depth = (string(argv[3]) == string("true"));

  if(g_inverse_depth){
  	output_log = "inv_"+output_log;
  	ROS_WARN("now play with inverse_depth!");
  }
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

  vector<double> v_d;

  int r, c;

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1);
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image);
        // accumulate_data(cv_ptrD->image);
        r = cv_ptrD->image.rows/2;
        c = cv_ptrD->image.cols/2;
        double dis = cv_ptrD->image.at<unsigned short>(r,c)*0.001;
		if(dis >0.5){

			if(g_inverse_depth)
				dis = 1./dis;

			// cout<<"inx: "<<inx<<endl;
			v_d.push_back(dis);
		}
      }
      if(!ros::ok())
      	break;
  }
  	vector<double> v = v_d;
  	double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
  	double m =  sum / v.size();

	double accum = 0.0;
	std::for_each (std::begin(v), std::end(v), [&](const double d) {
	    accum += (d - m) * (d - m);
	});

	double stdev = sqrt(accum / (v.size()-1));

	// output data
	ofstream ouf(output_log.c_str());

	ouf<<m<<" "<<stdev<<endl;
	for(int i=0; i<v.size(); i++)
		ouf<<v[i]<<" ";
	ouf<<endl;

	ouf.close();

  return ;
}
