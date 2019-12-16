/*
	
	Dec. 15, 2019, He Zhang, hzhang8@vcu.edu 

	estimate std for a bunch of grid points   

*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
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

struct loc{
	int r,c; 
	vector<double> dis; 
	vector<double> std;
};

// range of interest 
// int lc = 100;  int rc = 550;
// int ur = 80;  int lr = 440;

string out_log("output.log");
string cov_dir("./");  

void central_point_std(); 
void grid_points_std(); 

bool output_func(vector<loc>& pt_stats);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grid_points_std");
  
  ros::NodeHandle nh; 

  ROS_INFO("./grid_point_std [cov_dir] [output.log]");

  string bagfile = "";
  if(argc >= 2) 
    cov_dir = argv[1]; 

  if(argc >=3)
  	out_log = argv[2]; 

  central_point_std(); // compute the central point's std 

  return 0; 
}

void central_point_std()
{
	vector<double> v_std; 
	vector<double> v_dis; 

	// to be the same with depth_cov.cpp
	double MAX_STD = 0.2; // 5; //2.2; // so far 
	double MAX_DIS = 7.5; 
	int MAX_N = 65535; 

	int cr,cc;
	// 
	for(int d=60; d<=700; d+=20){
		stringstream ss, ss1 ;
		ss << cov_dir<< "/"<<d<<"cm.exr"; 
		ss1 << cov_dir<<"/mu_"<<d<<"cm.png"; 
		cv::Mat cov_img = cv::imread(ss.str().c_str(), IMREAD_ANYCOLOR | IMREAD_ANYDEPTH); 
		cv::Mat mean_img = cv::imread(ss1.str().c_str(), -1); 
		// cv::Mat gray; 
		// cv::cvtColor(cov_img, gray, CV_BGR2GRAY); 
		if(!cov_img.data || !mean_img.data){
			cout<<"grid_points_std.cpp: no data obtained from cov image"<<endl; 
			return ; 
		}

		int r = cov_img.rows/2; 
		int c = cov_img.cols/2; 
		cr = r; cc = c;
		// double std = cov_img.at<unsigned short>(r,c)*MAX_STD/MAX_N; 
		// double std = cov_img.at<float>(r,c); 
		// double std = cov_img.at<float>(r,c); 
		float std = getMean<float>(cov_img, r,c,1); 
		// double dis = mean_img.at<unsigned short>(r,c)*MAX_DIS/MAX_N; 
		unsigned short u_dis = getMean<unsigned short>(mean_img, r, c ,1);
		double dis = u_dis*MAX_DIS/MAX_N;

		// cout<<"d: "<<d<<" dis: "<<dis<<" img_mu: "<<(int)mean_img.at<unsigned short>(r,c)<<" std: "<<std
		// <<" img_cov: "<<(float)cov_img.at<float>(r,c)<< endl;

		cout<<"d: "<<d<<" dis: "<<dis<<" img_mu: "<<(int)u_dis<<" std: "<<std<<endl;

		if(dis > 0.5 && std > 0){
			v_std.push_back(std); 
			v_dis.push_back(dis); 
		}else{
			v_std.push_back(0); 
			v_dis.push_back(0); 
		}
	}
	loc L; L.r= cr; L.c=cc;
	L.dis.swap(v_dis); L.std.swap(v_std); 
	vector<loc> sv;
	sv.push_back(L); 
	output_func(sv);
	return ; 
}

void grid_points_std()
{

}

bool output_func(vector<loc>& pt_stats)
{
	ofstream ouf(out_log.c_str());
	if(!ouf.is_open()){
		cerr<<"grid_points_std.cpp: failed to open file: "<<out_log<<endl; 
		return false; 
	}


	for(int i=0; i<pt_stats.size(); i++){
		loc& l = pt_stats[i]; 
		ouf<<l.r<<"\t"<<l.c<<"\t"; 
		for(int m=0; m<l.dis.size(); m++){
			ouf<<l.dis[m]<<"\t";
		}
		for(int n=0; n<l.std.size(); n++){
			ouf<<l.std[n]<<"\t";
		}
	}
	ouf.close(); 
	return true; 
}