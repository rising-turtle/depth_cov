/*
	June 7, 2020, He Zhang, hzhang8@vcu.edu

	Monte Carlo sampling to compute depth/inverse-depth standard deviation for each pixel

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
#include "get_data.hpp"

using namespace std;
using namespace cv;

#define SQ(x) (((x)*(x)))

struct loc{
	int r,c;
	vector<double> dis;
};

struct MU{
	MU(){mean=0; num=0;}
	double sum(){return mean*num;}
	double mean;
	int num;
};

// range of interest
int lc = 100;
int rc = 550;
int ur = 80;
int lr = 440;

vector<vector<double>> v_dpts; // store repeated depth values
vector<MU> v_dpt_mean; // store mean depth value

string g_mt_file("inv_monte_carlo.png");
string gmm_img("inv_gmm.png");
string ori_img("inv_ori.png");

string output_gt_exr("output_std.exr");
string output_gt_img("output_std.png");
string output_raw_dpt_img("test.png");

void init();
void processBagfile(string bagfile);

void accumulate_data(const cv::Mat& dpt); // store depth data
void compute_statics(cv::Mat& G);
cv::Mat covert_to_color(cv::Mat& );
double rmse_diff(cv::Mat& d1, cv::Mat& d2);

bool g_inv_depth = false; // true

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "depth_distribute");
  ros::NodeHandle nh;

  ROS_INFO("./depth_distribute [bagfile] [output_std.exr] [output_std.png] [output_dpt.png]");

  string bagfile = "";
  if(argc >= 2)
    bagfile = argv[1];
  if(argc >= 3)
  	output_gt_exr = argv[2];
  if(argc >= 4)
  	output_gt_img = argv[3];
  if(argc >= 5)
  	output_raw_dpt_img = argv[4];

  processBagfile(bagfile);

  return 0;
}

void processBagfile(string bagfile)
{
  std::vector<std::string> topics;
  string dpt_tpc = "/cam0/depth";
  string rgb_tpc = "/cam0/color";
  topics.push_back(dpt_tpc);
  topics.push_back(rgb_tpc);

  rosbag::Bag bag;
  bag.open(bagfile, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // for extract cv::mat
  cv_bridge::CvImageConstPtr cv_ptrD;

  int cnt = 0;

  init();

  static bool first = true;
  cv::Mat prd;
  cv::Mat gmm;
  BOOST_FOREACH(rosbag::MessageInstance const m, view){

	 if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc){
        // receive a dpt image
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1);

        if(first){

        	// save this frame
        	cv::imwrite(output_raw_dpt_img.c_str(), cv_ptrD->image);

        	first = false;
        	usleep(1000);
        	continue;
        }
        accumulate_data(cv_ptrD->image);
        // cout<<"depth_distribute.cpp: show "<<++cnt<<" depth data!"<<endl;
        // waitKey(20);
      }
      if(!ros::ok())
      	break;
  }

  if(ros::ok()){
  	cv::Mat G;
  	compute_statics(G);

  	// double rmse_prd = rmse_diff(G, prd);
  	// double rmse_gmm = rmse_diff(G, gmm);

  	// ROS_INFO("gmm_depth.cpp: rmse_prd: %lf", rmse_prd);
  	// ROS_INFO("gmm_depth.cpp: rmse_gmm: %lf", rmse_gmm);

  	Mat mt_img = covert_to_color(G);
  	imwrite(output_gt_exr.c_str(), G);
  	imwrite(output_gt_img.c_str(), mt_img);
  	// imwrite(g_mt_file.c_str(), mt_img);

  	// Mat cm_img0;
  	// applyColorMap(MU, cm_img0, COLORMAP_JET);
  	// imshow("mean_depth: ", MU);
    // waitKey(2000);
  }


  return ;
}

void compute_statics(cv::Mat& G)
{
	int row = lr - ur + 1;
	int col = rc - lc + 1;
	// G = cv::Mat(row, col, CV_16UC1);
	G = cv::Mat(row, col, CV_32FC1);
	int i=-1;
	int N = row*col;
	vector<double> v_std(N, 0);
	vector<double> v_mean(N, 0);
	double max_std = 0;

	for(int r=0; r<row; r++)
	for(int c=0; c<col; c++)
	{
		++i;
		// compte mean and std
		vector<double>& v = v_dpts[i];
		if(v.size() <= 100){
			continue;
		}
		double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
		double m =  sum / v.size();

		double accum = 0.0;
		std::for_each (std::begin(v), std::end(v), [&](const double d) {
		    accum += (d - m) * (d - m);
		});

		double stdev = sqrt(accum / (v.size()-1));
		v_std[i] = stdev;
		v_mean[i] = m;
		if(stdev > max_std)
			max_std = stdev;
	}

	if(max_std <= 0) {
		cerr <<"what? max_std: "<<max_std<<endl;
		return ;
	}else{
		cout<<" max_std: "<<max_std<<endl;
	}

	i = -1;
	double ratio;
	for(int r=0; r<row; r++)
	for(int c=0; c<col; c++)
	{
		++i;
		G.at<float>(r,c) = v_std[i];
	}
	return ;
}

void accumulate_data(const cv::Mat& dpt)
{
	double dis;

	int inx = -1;
	// cout<<" v_dpts.size(): "<<v_dpts.size()<<endl;
	for(int r=ur; r<=lr; r++)
	for(int c=lc; c<=rc; c++)
	{
		dis = dpt.at<unsigned short>(r,c)*0.001;
		++inx;
		if(dis >0.5){
			// cout<<"inx: "<<inx<<endl;
			if(g_inv_depth)
				dis = 1./dis; // for inverse depth
			v_dpts[inx].push_back(dis);
			v_dpt_mean[inx].mean = (dis+v_dpt_mean[inx].sum())/(++v_dpt_mean[inx].num);
		}
	}
}

double rmse_diff(cv::Mat& d1, cv::Mat& d2)
{
	unsigned int cnt = 0;
	double sum_se = 0;

	for(int r=0; r<d1.rows; r++)
	for(int c=0; c<d1.cols; c++){

		double std1 = d1.at<float>(r,c);
		double std2 = d2.at<float>(r,c);

		if(std1 == 0 || std2 == 0)
			continue;
		sum_se += SQ(std1 - std2);
		++cnt;
	}
	double rmse = 0;
	if(cnt > 0) rmse = sqrt(sum_se/cnt);
	return rmse;
}


void init()
{
	for(int r=ur; r<=lr; r++)
	for(int c=lc; c<=rc; c++)
	{
		v_dpts.push_back(vector<double>());
		v_dpt_mean.push_back(MU());
	}
}

cv::Mat covert_to_color(cv::Mat& d)
{
	cv::Mat color= cv::Mat(d.rows, d.cols, CV_8UC1, Scalar(0));

	double MAX_STD = 0.055;
	double MAX_INV_STD = 0.01; //0.004;

	for(int r=0; r<d.rows; r++)
	for(int c=0; c<d.cols; c++){
		double std = d.at<float>(r,c);
		double ratio;
		if(g_inv_depth)
			ratio = std / MAX_INV_STD;
		else
			ratio = std / MAX_STD;
		ratio = ratio>1? 1.:ratio;
		color.at<unsigned char>(r,c) = (unsigned char)( ratio * 255);
	}

	Mat cm_img0;
  	applyColorMap(color, cm_img0, COLORMAP_HOT);
  	// imwrite(out_img, G);

	return cm_img0;
}
