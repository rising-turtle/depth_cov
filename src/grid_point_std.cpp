/*
	
	Dec. 15, 2019, He Zhang, hzhang8@vcu.edu 

	estimate std for a bunch of grid points   

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
vector<bool> v_dpt_valid; // decide whether this point's depth value is valid 

string out_img("output.png"); 

void init(); 

void processBagfile(string bagfile); 

void accumulate_data(const cv::Mat& dpt); // store depth data  
void find_valid(); // find valid points
void compute_statics(cv::Mat& G);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "depth_cov");
  
  ros::NodeHandle nh; 

  ROS_INFO("./depth_cov [bagfile] [*.png]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 

  if(argc >=3)
  	out_img = argv[2]; 

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

  init();

  // covert 16uc1 to 8uc1
  // dis = u16_d * 0.001
  // u8_d = dis / 7. * 255
  double scale = 0.001/7.*255; 

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 
        // imwrite(d_dpt + "/"+tt.str() +".png", cv_ptrD->image); 
        accumulate_data(cv_ptrD->image); 

        // Holds the colormap version of the image:
        Mat cm_img0;
        Mat psudo_grey; 
        // covert to 8UC1 
        convertScaleAbs(cv_ptrD->image, psudo_grey, scale); 
        // Apply the colormap:
        applyColorMap(psudo_grey, cm_img0, COLORMAP_JET);

        // imshow("dpt_file", cv_ptrD->image); 
        imshow("pysudo dpth", cm_img0);
        cout<<"depth_cov.cpp: show "<<++cnt<<" depth data!"<<endl;
        waitKey(20); 
      }
      if(!ros::ok())
      	break; 
  }
  if(ros::ok()){
  	find_valid();
  	cv::Mat G; 
  	compute_statics(G); 
  	Mat cm_img0;
  	applyColorMap(G, cm_img0, COLORMAP_JET);
  	imwrite(out_img, cm_img0); 
  	imshow("statistic pysudo dpth", cm_img0);
    waitKey(3000); 
  }
   

  return ; 
}


void compute_statics(cv::Mat& G){

	int row = lr - ur + 1; 
	int col = rc - lc + 1;
	G = cv::Mat(row, col, CV_8UC1);  
	int i=-1; 
	int N = row*col; 
	vector<double> v_std(N, 0); 
	double max_std = 0; 

	double MAX_STD = 0.5; //2.2; // so far 

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
		ratio = v_std[i]/MAX_STD; 
		if(ratio > 1.) ratio = 1.;
		G.at<unsigned char>(r,c) = (unsigned char)(ratio*255);
	}
	return ; 
}

void find_valid(){ // find valid points

	double dis; 

	int inx = -1;
	int ws = 3; 
	int COL = rc-lc+1; 
	int thre_num = (int)((ws*2+1)*(ws*2+1)/3);
	// cout<<" v_dpts.size(): "<<v_dpts.size()<<endl;  
	for(int r=ur; r<=lr; r++)
	for(int c=lc; c<=rc; c++)
	{
		++inx;
		int val_num = 0; 
		int inv_num = 0; 
		double max_dis = -1; 
		double min_dis = 10000; 
		// work in the local window 
		for(int rr=r-ws; rr<=r+ws; ++rr){
		for(int cc=c-ws; cc<=c+ws; ++cc){
			if(rr<ur || rr>lr || cc<lc || cc>rc)
				continue; 

			int cur_inx = (rr-ur)*COL + (cc-lc+1); 
			dis = v_dpt_mean[cur_inx].mean; 

			if(dis <= 0.5) {
				inv_num++
				continue; 
			}
			val_num++;
			if(max_dis < dis) max_dis = dis; 
			if(min_dis > dis) min_dis = dis;

		}
		}

		if(inv_num >= thre_num) 
			v_dpt_valid[inx] = false; 
		else if(max_dis - min_dis > 0.2){
			v_dpt_valid[inx] = false;
		}
	}

}
void accumulate_data(const cv::Mat& dpt){
	
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
			v_dpts[inx].push_back(dis); 
			v_dpt_mean[inx].mean = (dis+v_dpt_mean[inx].sum())/(++v_dpt_mean[inx].num);
		}
	}
}

void init(){

	for(int r=ur; r<=lr; r++)
	for(int c=lc; c<=rc; c++)
	{
		v_dpts.push_back(vector<double>()); 
		v_dpt_mean.push_back(MU());
		v_dpt_valid.push_back(true);
	}
}