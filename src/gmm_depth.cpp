/*
	Dec. 16, 2019, He Zhang, hzhang8@vcu.edu 

	compute and show the estimated depth std with and w/o gmm 

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

struct poly{
	poly(double para[3]){
		a1 = para[0]; a2 = para[1]; a3 = para[2]; 
	}
	poly(){}
	double y(double x){
		return (a1*x*x + a2*x+a3);
	}
	double a1,a2,a3;
	int r,c; 
};

// range of interest 
int lc = 100; 
int rc = 550;
int ur = 80; 
int lr = 440;

vector<vector<double>> v_dpts; // store repeated depth values 
vector<MU> v_dpt_mean; // store mean depth value 

string mt_img("monte_carlo.png"); 
string gmm_img("gmm.png");
string ori_img("ori.png");
string grid_point_fname("");
bool is_grid_pts_available = false;
vector<poly> v_grid_poly;   

void init(); 
bool init_grid_pt();

void processBagfile(string bagfile); 

void accumulate_data(const cv::Mat& dpt); // store depth data  
void compute_statics(cv::Mat& G, cv::Mat&);

cv::Mat predict_sigma(const cv::Mat& dpt);
cv::Mat predict_grid_point(const cv::Mat& dpt); 
cv::Mat gmm_sigma(const cv::Mat& dpt, cv::Mat& predict_sigma);
cv::Mat covert_to_color(cv::Mat& );

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gmm_depth");
  
  ros::NodeHandle nh; 

  ROS_INFO("./gmm_depth [bagfile] [grid_point.log]");

  string bagfile = "";
  if(argc >= 2) 
    bagfile = argv[1]; 
	
  if(argc >= 3)
  	grid_point_fname= argv[2]; 

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

  is_grid_pts_available = init_grid_pt(); 

  // covert 16uc1 to 8uc1
  // dis = u16_d * 0.001
  // u8_d = dis / 7. * 255
  double scale = 0.001/7.*255; 

  static bool first = true; 
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
      if(m.getTopic() == dpt_tpc || ("/"+m.getTopic()) == dpt_tpc)
      {
        // receive a dpt image
        sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
        cv_ptrD = cv_bridge::toCvShare(simage, sensor_msgs::image_encodings::TYPE_16UC1); 

        if(first){

        	// predict sigma 
        	cv::Mat prd; 
        	if(is_grid_pts_available){
        		prd = predict_grid_point(cv_ptrD->image);
        	}else{
        		prd = predict_sigma(cv_ptrD->image);
        	}

        	// gmm 
        	cv::Mat gmm = gmm_sigma(cv_ptrD->image, prd); 

        	// convert to pysudo color image 
        	cv::Mat prd_img = covert_to_color(prd); 
        	cv::Mat gmm_img = covert_to_color(gmm); 

        	// save them 
        	if(is_grid_pts_available){
        		cv::imwrite("grid_prd_img.png", prd_img);
        	}else
        		cv::imwrite("prd_img.png", prd_img); 
        	cv::imwrite("gmm_img.png", gmm_img); 

        	// cv::imwrite("prd_data.exr", prd);
        	// cv::imwrite("gmm_data.exr", gmm); 

        	first = false; 
        	usleep(1000); 
        	continue; 
        }
        accumulate_data(cv_ptrD->image); 
        cout<<"depth_cov.cpp: show "<<++cnt<<" depth data!"<<endl;
        // waitKey(20); 
      }
      if(!ros::ok())
      	break; 
  }

  if(ros::ok()){
  	cv::Mat G, MU; 
  	compute_statics(G, MU); 
  	Mat mt_img = covert_to_color(G); 
  	imwrite("mt_img.png", mt_img); 
  	// Mat cm_img0;
  	// applyColorMap(MU, cm_img0, COLORMAP_JET); 
  	imshow("mean_depth: ", MU);
    waitKey(5000); 
  }
   

  return ; 
}

cv::Mat predict_grid_point(const cv::Mat& dpt)
{
	int cols = rc - lc + 1; 
	int rows = lr - ur + 1; 
	int GRID_SIZE = 10; // equal to that in grid_point_std.cpp 
	cv::Mat sigma_img(rows, cols, CV_32FC1, Scalar(0.)); 

	// 35*44 = 1540, grid point 
	int X = 44; 
	int Y = 35; 

	// central point's parameters
	double para[3]={0.00155816, -0.00362021, 0.00452812};
	poly central_pt_predictor(para); 

	for(int r=0; r<rows; r++)
	for(int c=0; c<cols; c++){
		double dis = dpt.at<unsigned short>(r+ur, c+lc)*0.001; 
		double sigma; 
		if(dis <= 0.5) sigma = 0; 
		else{
			// find out neighboring 
			int lx = r/GRID_SIZE; 
			int ly = r/GRID_SIZE; 

			// weighted 
			double sum_w = 0; 
			vector<double> weights; 
			vector<double> v_std; 
			for(int y=ly-1; y<=ly; y++)
				for(int x=lx-1; x<=lx; x++){
					if(x<0 || x>=X || y<0 || y>=Y)
						continue; 
					int inx = y*X + x; 
					double std = v_grid_poly[inx].y(dis);
					double w = fabs(r-(y+1)*GRID_SIZE) + fabs(c-(x+1)*GRID_SIZE);
					weights.push_back(w);
					v_std.push_back(std); 
					sum_w += w;
				}
			if(sum_w == 0){
				// use central point instead
				sigma = central_pt_predictor.y(dis);
			}else{
				sigma = 0; 
				for(int i=0; i<v_std.size(); i++){
					sigma += (1-weights[i]/sum_w)*v_std[i];
				}
			}
		}
		sigma_img.at<float>(r,c) = sigma;
	}	

}
cv::Mat predict_sigma(const cv::Mat& dpt)
{
	int cols = rc - lc + 1; 
	int rows = lr - ur + 1; 
	// cv::Mat sigma_img(rows, cols, CV_8UC1, Scalar(0)); 
	cv::Mat sigma_img(rows, cols, CV_32FC1, Scalar(0.)); 
	double MAX_SIGMA = 0.055;
	
	// central point's parameters
	double para[3]={0.00155816, -0.00362021, 0.00452812};
	poly predictor(para); 

	for(int r=0; r<rows; r++)
	for(int c=0; c<cols; c++){
		double dis = dpt.at<unsigned short>(r+ur, c+lc)*0.001; 
		double sigma = predictor.y(dis);
		if(dis <= 0.5) sigma = 0; 
		// double ratio = (sigma/MAX_SIGMA); 
		// ratio = ratio > 1? 1.:ratio; 
		// sigma_img.at<unsigned char>(r,c) = (unsigned char)(ratio*255); 
		sigma_img.at<float>(r,c) = sigma;
	}
	return sigma_img;

}

cv::Mat gmm_sigma(const cv::Mat& dpt, cv::Mat& predict_sigma)
{	
	cv::Mat gmm_sig = predict_sigma.clone();
	cv::Mat W = (Mat_<double>(3,3) << 1, 2, 1, 2, 4, 2, 1, 2, 1);

	for(int r=1; r<predict_sigma.rows-1; r++)
	for(int c=1; c<predict_sigma.cols-1; c++){

		double mu_z = 0; 
		double std_sq = 0;

		double sW = 0; 

		// find out valid point 
		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-5)
				continue;
			sW += W.at<double>(i,j);
		}

		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-5)
				continue;
			mu_z += W.at<double>(i,j)*mu_ij/sW; 
			std_sq += W.at<double>(i,j)*(SQ(std_ij)+SQ(mu_ij))/sW;
		}
		std_sq = std_sq - SQ(mu_z); 
		gmm_sig.at<float>(r,c) = sqrt(std_sq); 
	}
	return gmm_sig;
}


void compute_statics(cv::Mat& G, cv::Mat& MU){

	int row = lr - ur + 1; 
	int col = rc - lc + 1;
	// G = cv::Mat(row, col, CV_16UC1);  
	G = cv::Mat(row, col, CV_32FC1);  
	MU = cv::Mat(row, col, CV_16UC1);
	int i=-1; 
	int N = row*col; 
	vector<double> v_std(N, 0); 
	vector<double> v_mean(N, 0); 
	double max_std = 0; 

	double MAX_STD = 0.2; //2.2; // so far 
	double MAX_DIS = 7.5; 
	int MAX_N = 65535; 

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
		ratio = v_std[i]/MAX_STD; 
		if(ratio > 1.) ratio = 1.;
		// G.at<unsigned char>(r,c) = (unsigned char)(ratio*255);
		// G.at<unsigned short>(r,c) = (unsigned short)(ratio*MAX_N);
		G.at<float>(r,c) = v_std[i]; 
		ratio = v_mean[i]/MAX_DIS; 
		if(ratio > 1.) ratio = 1;
		// MU.at<unsigned char>(r,c) = (unsigned char)(ratio*MAX_N);
		MU.at<unsigned short>(r,c) = (unsigned short)(ratio*MAX_N);
	}
	return ; 
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

bool init_grid_pt()
{
	ifstream inf(grid_point_fname.c_str()); 

	if(!inf.is_open()){
		cout<<"gmm_depth: cannot found grid_point file: "<<grid_point_fname<<endl; 
		return false; 
	}

	char buf[4096];
	while(inf.getline(buf, 4096)){
		poly gp; 
		double lr, lc; 
		sscanf(buf, "%lf %lf %lf %lf %lf", &lr, &lc, &gp.a1, &gp.a2, &gp.a3); 
		gp.r = lr; 
		gp.c = lc; 
		if(v_grid_poly.size() < 10){
			printf("read %d %d %f %f %f \n", lr, lc, gp.a1, gp.a2, gp.a3);
		}
		v_grid_poly.push_back(gp);
	}

	printf("read %d grid points\n", v_grid_poly.size()); 

	return true; 
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

	for(int r=0; r<d.rows; r++)
	for(int c=0; c<d.cols; c++){
		double std = d.at<float>(r,c); 
		double ratio = std / MAX_STD;
		ratio = ratio>1? 1.:ratio;
		color.at<unsigned char>(r,c) = (unsigned char)( ratio * 255);
	}

	Mat cm_img0;
  	applyColorMap(color, cm_img0, COLORMAP_HOT);
  	// imwrite(out_img, G); 

	return cm_img0;
}