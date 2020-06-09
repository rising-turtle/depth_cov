/*
	June 9, 2020, He Zhang, hzhang8@vcu.edu 

	optimize the lambda/scale parameter 
	
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

string test_file("test.png"); 
string true_file("output_std.exr"); 

void do_it(); 

struct lambdaResidual {
	lambdaResidual(cv::Mat& img, cv::Mat& std_img){
		dpt_img = img; 
		v_std = fromMatToVector<float>(dpt_img); 
		prd_img = predict_sigma(dpt_img);
	}
	//}

	double loss(double delta_lambda, double lambda, double sigma, double local_sigma=1){
		// return log(SQ(delta_lambda)*(SQ(lambda)+1));
		double scale = 1; // 700000; // SQ(0.001); // SQ(0.01);
		// return log(SQ(delta_lambda)*SQ(lambda)/(scale*SQ(local_sigma))+1);
		// return log(SQ(delta_lambda)*SQ(lambda)/(scale*SQ(local_sigma)) + 1);
		// return scale*log(SQ(delta_lambda)*SQ(lambda) + 1);
		return (SQ(delta_lambda)*SQ(lambda));
		// return scale*log(SQ(delta_lambda)*SQ(lambda)/(SQ(local_sigma)) + 1);
	}


	bool operator()(const double* params, double* residual) const {

		double lambda = params[0]; 

		cv::Mat gmm_img = gmm_bilateral_sigma(dpt_img, prd_img, lambda); 
		vector<float> gmm_std = fromMatToVector<float>(gmm_img); 

		assert(gmm_std.size() == v_std.size()); 

		double res = 0; 
		for(int i=0; i<gmm_std.size(); i++){
			if(gmm_std[i] > 0){
				res += SQ(gmm_std[i] - v_std[i]); 
			}
		}
		residual[0] = sqrt(res); 

		return true; 
	}
	cv::Mat gmm_bilateral_sigma(const cv::Mat& dpt, const cv::Mat& predict_sigma, double lambda); 
	cv::Mat predict_sigma(const cv::Mat& dpt);
	template<typename T> 
	vector<T> fromMatToVector(cv::Mat& m){
		int N = m.rows*m.cols; 
		std::vector<T> v(N);
		for(int r=0; r<m.rows; r++)
			for(int c=0; c<m.cols; c++){
				v[r*m.cols+c] = m.at<T>(r,c);
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
  ros::init(argc, argv, "solve_lambda");
  
  ros::NodeHandle nh; 

  ROS_INFO("./solve_lambda [test_file.png] [true_std_file.exr]");

  string bagfile = "";
  if(argc >= 2) 
    test_file = argv[1]; 
	
  if(argc >= 3)
  	true_file= argv[2]; 

  do_it(); 

  return 0; 
}

void do_it()
{
	cv::Mat std_img = cv::imread(true_file.c_str(), IMREAD_ANYCOLOR | IMREAD_ANYDEPTH); 
	cv::Mat dpt_img = cv::imread(test_file.c_str(), -1); 

	if(std_img.empty() || dpt_img.empty()){
		ROS_INFO("solve_lambda.cpp: failed to read std_img or dpt_img"); 
		return ; 
	}

	double lambda = 1; 

	// solve problem 
	Problem problem;
  	problem.AddResidualBlock(new ceres::NumericDiffCostFunction<lambdaResidual, ceres::FORWARD, 1, 1>(new lambdaResidual(dpt_img, std_img)), 
  							NULL, &lambda); 

  	Solver::Options options;
  	options.max_num_iterations = 250;
  	options.linear_solver_type = ceres::DENSE_QR;
  	options.minimizer_progress_to_stdout = true;

  	Solver::Summary summary;
  	Solve(options, &problem, &summary);
  	std::cout << summary.BriefReport() << "\n";
  	std::cout << "Initial lambda: " << lambda << "\n";
  	std::cout << "Final   lambda: " << lambda << "\n";
  	return 0;
}


cv::Mat lambdaResidual::gmm_bilateral_sigma(const cv::Mat& dpt, const cv::Mat& predict_sigma, double lambda)
{
	cv::Mat gmm_sig = predict_sigma.clone();
	cv::Mat W = (Mat_<double>(3,3) << 4, 1, 4, 1, 0, 1, 4, 1, 4);

	for(int r=1; r<predict_sigma.rows-1; r++)
	for(int c=1; c<predict_sigma.cols-1; c++){

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

		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			// if((i==2 && j==0) || (i==0 && j==2) || (i==2 && j==2) || (i==0 && j==0))
			//	continue;
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001; 
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-5)
				continue;
			mu_ij = 1./mu_ij; 
			double w = -W.at<double>(i,j)/2. - lambda * loss(mu_ij - mu_d, mu_d, std_d, local_std);  
			w = exp(w);
			// mu_z += W.at<double>(i,j)*mu_ij/sW; 
			// std_sq += W.at<double>(i,j)*(SQ(std_ij)+SQ(mu_ij))/sW;
			mu_z += w * mu_ij / sW; 
			std_sq += w*(SQ(std_ij)+SQ(mu_ij))/sW;
		}
		std_sq = std_sq - SQ(mu_z); 
		gmm_sig.at<float>(r,c) = sqrt(std_sq); 
	}
	return gmm_sig;
}



cv::Mat lambdaResidual::predict_sigma(const cv::Mat& dpt)
{
	int cols = rc - lc + 1; 
	int rows = lr - ur + 1; 

	// cv::Mat sigma_img(rows, cols, CV_8UC1, Scalar(0)); 
	cv::Mat sigma_img(dpt.rows, dpt.cols, CV_32FC1, Scalar(0.)); 	

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