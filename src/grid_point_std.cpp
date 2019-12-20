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

string prefix("");
string g_central_pt_log("central_point_std.log");
string g_grid_pt_log("grid_point_std.log");
string cov_dir("./");  

bool g_inverse_depth=false; 

vector<loc> init_loc(int GRID_SIZE); // grid size 
void central_point_std(); 
void grid_points_std(); 

bool output_func(vector<loc>& pt_stats, string out_log);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grid_points_std");
  
  ros::NodeHandle nh; 

  ROS_INFO("./grid_point_std [cov_dir] [output.log]");

  string bagfile = "";
  if(argc >= 2) 
    cov_dir = argv[1]; 

  if(argc >=3)
  	g_central_pt_log = argv[2];

  if(argc >=4){
  	g_inverse_depth = (string(argv[3]) == string("true")); 
  } 

  if(g_inverse_depth){
  	ROS_WARN("grid_point_std.cpp: now play with inverse depth !!!!");
  	prefix = "inv_";
  	g_central_pt_log = prefix + g_central_pt_log;
  	g_grid_pt_log = prefix + g_grid_pt_log;
  }else
  	ROS_WARN("grid_points_std.cpp: in depth !");

  // central_point_std(); // compute the central point's std 
  grid_points_std();

  return 0; 
}

void grid_points_std()
{
	// to be the same with depth_cov.cpp
	double MAX_STD = 0.2; // 5; //2.2; // so far 
	double MAX_DIS = 7.5; 
	double MAX_INV_DIS = 1/0.6;
	int MAX_N = 65535; 

	int GRID_SIZE = 10; 
	vector<loc> v_loc = init_loc(GRID_SIZE);

	for(int d=60; d<=700; d+=20){
		stringstream ss, ss1 ;
		ss << cov_dir<< "/"<<prefix<<d<<"cm.exr"; 
		ss1 << cov_dir<<"/"<<prefix<<"mu_"<<d<<"cm.png"; 
		cv::Mat cov_img = cv::imread(ss.str().c_str(), IMREAD_ANYCOLOR | IMREAD_ANYDEPTH); 
		cv::Mat mean_img = cv::imread(ss1.str().c_str(), -1); 
		// cv::Mat gray; 
		// cv::cvtColor(cov_img, gray, CV_BGR2GRAY); 
		if(!cov_img.data || !mean_img.data){
			cout<<"grid_points_std.cpp: no data obtained from cov image"<<endl; 
			return ; 
		} 

		for(int i=0; i<v_loc.size(); i++){
			int r = v_loc[i].r; 
			int c = v_loc[i].c; 
			int w = GRID_SIZE/2;
			if(w > 3) w = 3;  

			float std = getMedian<float>(cov_img, r,c,w);
			unsigned short u_dis = getMedian<unsigned short>(mean_img, r,c,w); 
			double dis;
			if(g_inverse_depth)
				dis = u_dis * MAX_INV_DIS/MAX_N;
			else
				dis = u_dis * MAX_DIS/MAX_N; 

			if(!g_inverse_depth && dis > 0.5 && std > 0){
				v_loc[i].std.push_back(std); 
				v_loc[i].dis.push_back(dis); 
			}else if(g_inverse_depth && dis < (1./0.5) && std > 0)
			{
				v_loc[i].std.push_back(std); 
				v_loc[i].dis.push_back(dis); 
			}else{

			}
		}
		cout<<"handle data: "<<ss.str()<<endl; 
	}
	cout<<"v_loc.size(): "<<v_loc.size()<<" prepare to output"<<endl;
	// output
	output_func(v_loc, g_grid_pt_log);
	return ; 
}

vector<loc> init_loc(int GRID_SIZE) // grid size 
{
	vector<loc> v_loc; 
	// parameters in depth_cov 
	int lc = 100; 
	int rc = 550;
	int ur = 80; 
	int lr = 440;
	int rows = lr - ur + 1; 
	int cols = rc - lc + 1;

	for(int r=GRID_SIZE; r<(rows-GRID_SIZE/2); r+=GRID_SIZE)
	for(int c=GRID_SIZE; c<(cols-GRID_SIZE/2); c+=GRID_SIZE){
		loc C;
		C.r = r; 
		C.c = c; 
		v_loc.push_back(C); 
	}
	return v_loc;
}

void central_point_std()
{
	vector<double> v_std; 
	vector<double> v_dis; 

	// to be the same with depth_cov.cpp
	double MAX_STD = 0.2; // 5; //2.2; // so far 
	double MAX_DIS = 7.5; 
	int MAX_N = 65535; 
	double MAX_INV_DIS = 1/0.6;

	int cr,cc;
	// 
	for(int d=60; d<=700; d+=20){
		stringstream ss, ss1 ;
		ss << cov_dir<< "/"<<prefix<<d<<"cm.exr"; 
		ss1 << cov_dir<<"/"<<prefix<<"mu_"<<d<<"cm.png"; 
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
		double dis;
		if(g_inverse_depth){
			dis = u_dis*MAX_INV_DIS/MAX_N;
		}
		else{
			dis = u_dis*MAX_DIS/MAX_N;
		}

		// cout<<"d: "<<d<<" dis: "<<dis<<" img_mu: "<<(int)mean_img.at<unsigned short>(r,c)<<" std: "<<std
		// <<" img_cov: "<<(float)cov_img.at<float>(r,c)<< endl;

		cout<<"d: "<<d<<" dis: "<<dis<<" img_mu: "<<(int)u_dis<<" std: "<<std<<endl;

		if(!g_inverse_depth && dis > 0.5 && std > 0){
			v_std.push_back(std); 
			v_dis.push_back(dis); 
		}else if(g_inverse_depth && dis < (1./0.5) && std > 0)
		{
			v_std.push_back(std); 
			v_dis.push_back(dis); 
		}else{

		}
	}
	loc L; L.r= cr; L.c=cc;
	L.dis.swap(v_dis); L.std.swap(v_std); 
	vector<loc> sv;
	sv.push_back(L); 
	output_func(sv, g_central_pt_log);
	return ; 
}

bool output_func(vector<loc>& pt_stats, string out_log)
{
	ofstream ouf(out_log.c_str());
	if(!ouf.is_open()){
		cerr<<"grid_points_std.cpp: failed to open file: "<<out_log<<endl; 
		return false; 
	}

	for(int i=0; i<pt_stats.size(); i++){
		loc& l = pt_stats[i]; 
		ouf<<l.r<<"\t"<<l.c<<"\t"<<l.dis.size()<<"\t"; 
		for(int m=0; m<l.dis.size(); m++){
			ouf<<l.dis[m]<<"\t";
		}
		for(int n=0; n<l.std.size(); n++){
			ouf<<l.std[n]<<"\t";
		}
		ouf<<endl;
	}
	ouf.close(); 
	return true; 
}