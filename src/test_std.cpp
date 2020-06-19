/*
	June 19, 2020, He Zhang, hzhang8@vcu.edu 

	test why std is negative 
	
*/
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sys/stat.h>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <stdlib.h>
#include "get_data.hpp"
#include "ceres/ceres.h"

#define SQ(x) (((x)*(x)))

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std; 
using namespace cv; 

// range of interest 
int lc = 100; 
int rc = 550;
int ur = 80; 
int lr = 440;

void do_it(); 
cv::Mat covert_to_color(cv::Mat& );
double rmse_diff(cv::Mat& d1, cv::Mat& d2);

struct lambdaResidual {
	lambdaResidual(cv::Mat& img, cv::Mat& std_img){
		dpt_img = img; 
		v_std = fromMatToVector<float>(std_img); 
		prd_img = predict_sigma(dpt_img);
	}
	//}

	double loss(double delta_lambda, double lambda, double sigma, double local_sigma=1){
		// return log(SQ(delta_lambda)*(SQ(lambda)+1));
		double scale = 700000; // SQ(0.001); // SQ(0.01);
		// return log(SQ(delta_lambda)*SQ(lambda)/(scale*SQ(local_sigma))+1);
		// return log(SQ(delta_lambda) + 1);
		return log(SQ(delta_lambda)*SQ(lambda) + 1);
		// return (SQ(delta_lambda)*SQ(lambda));
		// return scale*log(SQ(delta_lambda)*SQ(lambda)/(SQ(local_sigma)) + 1);
	}


	bool operator()(const double* params, double* residual) const {

		double lambda = params[0]; 

		cv::Mat gmm_img = gmm_bilateral_sigma(dpt_img, prd_img, lambda); 
		vector<float> gmm_std = fromMatToVector<float>(gmm_img); 

		assert(gmm_std.size() == v_std.size()); 

		double res = 0; 
		int cnt = 0;
		for(int i=0; i<gmm_std.size(); i++){
			if(gmm_std[i] > 0){
				res += SQ(gmm_std[i] - v_std[i]);
				++cnt;  
			}
		}
		if(cnt > 0)
			residual[0] = 1000*sqrt(res/cnt); 
		else 
			residual[0] = 0; 

		return true; 
	}
	cv::Mat output_gmm_image(double lambda){
		return gmm_bilateral_sigma(dpt_img, prd_img, lambda); 
	}
	cv::Mat gmm_bilateral_sigma(const cv::Mat& dpt, const cv::Mat& predict_sigma, double lambda); 
	cv::Mat predict_sigma(const cv::Mat& dpt);
	template<typename T> 
	vector<T> fromMatToVector(cv::Mat& m){
		int N = m.rows*m.cols; 
		std::vector<T> v(N);
		int ii = 0; 
		for(int r=0; r<m.rows; r++)
			for(int c=0; c<m.cols; c++){
				v[ii++] = m.at<T>(r,c);
			}
		return v; 
	}

private:
	cv::Mat dpt_img; 
	cv::Mat prd_img; 
	vector<float> v_std; 
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_std");
  
  	ros::NodeHandle nh; 

	ROS_INFO("./test_std ");
	do_it(); 

  return 0; 
}

void do_it()
{
	double lambda = 17124.8;

	cv::Mat std_img = cv::imread("./tmp/7.exr", IMREAD_ANYCOLOR | IMREAD_ANYDEPTH); 
	cv::Mat dpt_img = cv::imread("./tmp/7_dpt.png", -1); 
	if(std_img.empty() || dpt_img.empty()){
		ROS_INFO("solve_lambda.cpp: failed to read std_img or dpt_img "); 
		return ; 
	}

	
	lambdaResidual * pp =  new lambdaResidual(dpt_img, std_img); 
	cv::Mat prd_img = pp->predict_sigma(dpt_img);
	pp->gmm_bilateral_sigma(dpt_img, prd_img, lambda); 

  	return 0;
}


cv::Mat lambdaResidual::gmm_bilateral_sigma(const cv::Mat& dpt, const cv::Mat& predict_sigma, double lambda)
{
	cv::Mat gmm_sig = predict_sigma.clone();
	cv::Mat W = (Mat_<double>(3,3) << 4, 1, 4, 1, 0, 1, 4, 1, 4);

	int tc = 430; 
	int tr = 262; 

	for(int r=tr; r<tr+1; r++)
	for(int c=tc; c<tc+1; c++){

		double mu_z = 0; 
		double std_sq = 0;

		double sW = 0; 

		double mu_d = dpt.at<unsigned short>(r+ur, c+lc)*0.001; 
		double std_d = predict_sigma.at<float>(r, c); 
		if(mu_d <= 0.5 || std_d <= 1e-7) {
			mu_d = getMean<unsigned short>(dpt, r+ur, c+lc, 3)*0.001;
			std_d = getMean<float>(predict_sigma, r, c, 3); 

			if(mu_d <= 0.5 || std_d <= 1e-7){
				gmm_sig.at<float>(r,c) = 0;
				continue; 
			}
		}

		mu_d = 1./mu_d; 

		int n_invalid = 1; 
		double local_std = 1;
		vector<double> vdpt;  

		if(r>=2 && r<predict_sigma.rows-2 && c>=2 && c<predict_sigma.cols-2){
		// find out local std 
		for(int i=0; i<5; i++)
		for(int j=0; j<5; j++){
			double mu_ij = dpt.at<unsigned short>(r+ur+i-2, c+lc+j-2)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-2, c+j-2);
			if(mu_ij <= 0.5 || std_ij <= 1e-7){
				n_invalid++;
				continue;
			}
			vdpt.push_back(1./mu_ij); 
		}

		if(vdpt.size() > 1)
		{
			double sum = std::accumulate(std::begin(vdpt), std::end(vdpt), 0.0);
			double m =  sum / vdpt.size();

			double accum = 0.0;
			std::for_each (std::begin(vdpt), std::end(vdpt), [&](const double d) {
			    accum += (d - m) * (d - m);
			});

			double stdev = sqrt(accum / (vdpt.size()-1));
			local_std = 2*n_invalid+stdev; // 1*n_invalid + stdev*10;
		}
		}

		// find out valid point 
		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			// if((i==2 && j==0) || (i==0 && j==2) || (i==2 && j==2) || (i==0 && j==0))
			//	continue;
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-7)
				continue;
			mu_ij = 1./mu_ij; 
			double w = -W.at<double>(i,j)/2. - lambda * loss(mu_ij - mu_d, mu_d, std_d, local_std);  
			// sW += W.at<double>(i,j);
			sW += exp(w); 
		}

		cout<<"sW: "<<sW<<endl; 

		if(sW == 0){
			gmm_sig.at<float>(r,c) = 0; 
			continue;
		}

		cout<<"before: mu_z: "<<mu_z<<" std_sq: "<<std_sq<<endl; 
		double sum_w = 0; 

		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			// if((i==2 && j==0) || (i==0 && j==2) || (i==2 && j==2) || (i==0 && j==0))
			//	continue;
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-7)
				continue;
			mu_ij = 1./mu_ij; 
			double w = -W.at<double>(i,j)/2. - lambda * loss(mu_ij - mu_d, mu_d, std_d, local_std);  
			w = exp(w);
			sum_w += w; 
			// mu_z += W.at<double>(i,j)*mu_ij/sW; 
			// std_sq += W.at<double>(i,j)*(SQ(std_ij)+SQ(mu_ij))/sW;
			mu_z += w * mu_ij / sW; 
			std_sq += w*(SQ(std_ij)+SQ(mu_ij))/sW;

			cout<<" i: "<<i<<" j: "<<j<<endl; 
			cout <<"w: "<<w<<" mu_z: "<<mu_z<<" sum_w: "<<sum_w<<" std_sq: "<<std_sq<<endl; 
			if(std_sq != std_sq)
				cout<<"what? mu_z: "<<mu_z<<" sW: "<<sW<<" w: "<<w<<" mu_ij: "<<mu_ij<<" std_ij: "<<std_ij<<endl;
		}
		cout<<"final: std_sq: "<<std_sq<<" mu_z: "<<mu_z<<" SQ(mu_z): "<< SQ(mu_z)<<endl; 
		std_sq = std_sq - SQ(mu_z); 

		if(std_sq < 0) {
			cout <<"what:  std_sq: " <<std_sq<<" r: "<<r<<" c: "<<c<<endl;
			gmm_sig.at<float>(r,c) = 0; 

		}else
			gmm_sig.at<float>(r,c) = sqrt(std_sq); 
	}
	return gmm_sig;
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
		if(std1!= std1 || std2!=std2){
			cout<<"what? std1: "<<std1<<" std2: "<<std2<<endl;

		}
		sum_se += SQ(fabs(std1 - std2)); 
		++cnt;  
	}
	double rmse = 0; 
	if(cnt > 0) rmse = sqrt(sum_se/cnt); 
	return rmse; 
}


cv::Mat lambdaResidual::predict_sigma(const cv::Mat& dpt)
{
	int cols = rc - lc + 1; 
	int rows = lr - ur + 1; 

	// cv::Mat sigma_img(rows, cols, CV_8UC1, Scalar(0)); 
	cv::Mat sigma_img(rows, cols, CV_32FC1, Scalar(0.)); 	

	for(int r=0; r<rows; r++)
	for(int c=0; c<cols; c++){
		double dis = dpt.at<unsigned short>(r+ur, c+lc)*0.001; 
		double sigma; 
		if(dis <= 0.5) sigma = 0; 
		else{
			sigma = 0.0005;
		}
		sigma_img.at<float>(r,c) = sigma;
	}
	return sigma_img;

}
