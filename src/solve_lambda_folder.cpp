/*
	June 19, 2020, He Zhang, hzhang8@vcu.edu

	optimize the lambda/scale parameter with data in a folder

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

// depth_sigma = y(depth)
struct poly{
	poly(double para[3]){
		a1 = para[0]; a2 = para[1]; a3 = para[2];
	}
	poly(){}
	double y(double x){
		if(x <= 0.75)
			return 0.0007;
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

string folder_name("./tmp");
int num(10);
double g_lambda = 1.;

bool g_is_inv_dis = false; // true; //false; // true

void show_result(double);
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

	double loss_ind(double delta_lambda, double lambda, double sigma, double local_sigma=1){
		// return log(SQ(delta_lambda)*(SQ(lambda)+1));
		double scale = 700000; // SQ(0.001); // SQ(0.01);
		// return log(SQ(delta_lambda)*SQ(lambda)/(scale*SQ(local_sigma))+1);
		return log(SQ(delta_lambda)*SQ(lambda)/(SQ(local_sigma))+1); // 806800 806800 3.08269e+06
		// return log(SQ(delta_lambda)/(SQ(local_sigma))+1); // 1.37847e+06

		// return SQ(delta_lambda)*SQ(lambda)/(SQ(local_sigma)); // 2.8068e+06

		// return log(SQ(delta_lambda)/((SQ(local_sigma)))+1);
		// return log(SQ(delta_lambda) + 1);
		// return log(SQ(delta_lambda)*SQ(lambda) + 1); // lambda = 181027,
		// return (SQ(delta_lambda)*SQ(lambda)); // lambda = 188327
		// return (SQ(delta_lambda)); // lambda = 188327
		// return log(SQ(delta_lambda)*SQ(lambda)/(SQ(local_sigma)) + 1);
	}

	double loss_d(double delta_dis, double dis, double dis_sigma, double local_sigma=1){
		// return SQ(delta_dis)/(SQ(dis_sigma)*SQ(local_sigma)); // not good
		// return SQ(delta_dis)/(SQ(dis)*SQ(local_sigma)); //
		return log(SQ(delta_dis)/(SQ(dis_sigma)*SQ(local_sigma))+1); // lambda = 1.2
		// return log(SQ(delta_dis)/(SQ(dis)*SQ(local_sigma))+1); // lambda = 1.2
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
  ros::init(argc, argv, "solve_lambda_folder");

  ros::NodeHandle nh;

  ROS_INFO("./solve_lambda_folder [folder] [number] [lambda] [depth/inverse_depth]");

  string bagfile = "";
  if(argc >= 2)
    folder_name = argv[1];

  if(argc >= 3)
  	num= atoi(argv[2]);

	if(argc >= 4){
		if(string(argv[3]) == string("depth")){
			cout <<" solve_lambda_folder: solve lambda for depth"<<endl;
			g_is_inv_dis = false;
		}else if(string(argv[3]) == string("inverse_depth")){
			cout <<" solve_lambda_folder: solve lambda for inverse depth"<<endl;
			g_is_inv_dis = true;
		}else{
			cout <<"solve_lambda_folder: unknown parameter argv[3]: "<<argv[3]<<endl;
			cout <<"default solve lambda for depth "<<endl;
		}
	}

  if(argc >= 5){
  	g_lambda = atof(argv[4]);
  	show_result(g_lambda);
  	return 0;
  }

  do_it();

  return 0;
}

void do_it()
{
	double lambda = 1;

	// solve problem
	Problem problem;
	for(int i=1; i<=num; i++){
		if(i%5 == 0) continue;
		stringstream tu_file, te_file;
		tu_file<<folder_name<<"/"<<i<<".exr";
		te_file<<folder_name<<"/"<<i<<"_dpt.png";
		cv::Mat std_img = cv::imread(tu_file.str().c_str(), IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);
		cv::Mat dpt_img = cv::imread(te_file.str().c_str(), -1);
		if(std_img.empty() || dpt_img.empty()){
			ROS_INFO("solve_lambda.cpp: failed to read std_img: %s or dpt_img: %s",tu_file.str().c_str(), te_file.str().c_str());
			return ;
		}
		lambdaResidual * pp =  new lambdaResidual(dpt_img, std_img);
		// cv::Mat pre_opt_img = pp->output_gmm_image(lambda);
  	//	problem.AddResidualBlock(new ceres::NumericDiffCostFunction<lambdaResidual, ceres::FORWARD, 1, 1>(new lambdaResidual(dpt_img, std_img)),
  	//						 new ceres::CauchyLoss(0.02), &lambda);
		problem.AddResidualBlock(new ceres::NumericDiffCostFunction<lambdaResidual, ceres::FORWARD, 1, 1>(new lambdaResidual(dpt_img, std_img)),
			 		  							 NULL, &lambda);

	}
	problem.SetParameterLowerBound(&lambda, 0, 0);
  	Solver::Options options;
  	options.max_num_iterations = 250;
  	options.linear_solver_type = ceres::DENSE_QR;
  	options.minimizer_progress_to_stdout = true;

  	Solver::Summary summary;
  	std::cout << "Initial lambda: " << lambda << "\n";
  	Solve(options, &problem, &summary);
  	std::cout << summary.BriefReport() << "\n";
  	std::cout << "Final   lambda: " << lambda << "\n";

  	show_result(lambda);
  	// cv::Mat pos_opt_img = pp->output_gmm_image(lambda);


  	// cv::Mat pre_opt_img_t = covert_to_color(pre_opt_img);
  	// cv::Mat pos_opt_img_t = covert_to_color(pos_opt_img);

  	// save image to see output
  	// cv::imwrite("pre_opt.png", pre_opt_img_t);
  	// cv::imwrite("pos_opt.png", pos_opt_img_t);

  	return 0;
}

void show_result(double lambda)
{
	for(int i=1; i<=num; i++){
  		if(i%5 != 0) continue;
		stringstream tu_file, te_file;
		tu_file<<folder_name<<"/"<<i<<".exr";
		te_file<<folder_name<<"/"<<i<<"_dpt.png";
		cv::Mat std_img = cv::imread(tu_file.str().c_str(), IMREAD_ANYCOLOR | IMREAD_ANYDEPTH);
		cv::Mat dpt_img = cv::imread(te_file.str().c_str(), -1);

		cv::Mat dpt_img_8u;
		dpt_img.convertTo(dpt_img_8u, CV_8U, 1./20.); // max 5m * 1000 /256 ~ 40
		cv::Rect roi(lc, ur, rc-lc+1, lr-ur+1);
		cv::Mat sub_dpt_8u(dpt_img_8u, roi);
		cv::Mat psu_depth;
		cv::applyColorMap(sub_dpt_8u, psu_depth, COLORMAP_JET);

		lambdaResidual * pp =  new lambdaResidual(dpt_img, std_img);
		cv::Mat pred_img = pp->predict_sigma(dpt_img);
		cv::Mat gmm_0_img = pp->output_gmm_image(0);
		// cv::Mat pre_opt_img = pp->output_gmm_image(1);
		cv::Mat pos_opt_img = pp->output_gmm_image(lambda);

		double rmse_gmm_0 = rmse_diff(gmm_0_img, std_img);
  		// double rmse_pre = rmse_diff(pre_opt_img, std_img);
  		double rmse_pos = rmse_diff(pos_opt_img, std_img);
  		cout <<"solve_lambda_folder.cpp: rmse for img: "<<i<<endl;
  		cout <<"rmse_gmm_0: "<<rmse_gmm_0<<endl;
  		// cout <<"rmse_pre: "<<rmse_pre<<endl;
  		cout <<"rmse_pos: "<<rmse_pos<<endl;

  		stringstream gmm_of, ext_gmm_of, pred_of, psu_of;
  		gmm_of<<folder_name<<"/"<<i<<"_gmm.png";
  		ext_gmm_of<<folder_name<<"/"<<i<<"_gmm_ext.png";
			pred_of << folder_name<<"/"<<i<<"_pred.png";
			psu_of<< folder_name <<"/"<<i<<"_psu_dpt.png";

		cv::Mat gmm_of_img = covert_to_color(gmm_0_img);
		cv::Mat ext_gmm_of_img = covert_to_color(pos_opt_img);
		cv::Mat pred_of_img = covert_to_color(pred_img);
  		// save image to see output
  		cv::imwrite(gmm_of.str().c_str(), gmm_of_img);
  		cv::imwrite(ext_gmm_of.str().c_str(), ext_gmm_of_img);
			cv::imwrite(pred_of.str().c_str(), pred_of_img);
			cv::imwrite(psu_of.str().c_str(), psu_depth);
	}

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
		if(g_is_inv_dis)
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

			if(g_is_inv_dis)
				vdpt.push_back(1./mu_ij);
			else
				vdpt.push_back(mu_ij);
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
			if(g_is_inv_dis)
				mu_ij = 1./mu_ij;
			// double w = -W.at<double>(i,j)/2. - lambda * loss(mu_ij - mu_d, mu_d, std_d, local_std);
			double w;
			if(g_is_inv_dis)
				w = -W.at<double>(i,j)/2. - lambda * loss_ind(mu_ij - mu_d, mu_d, std_d, local_std);
			else
				w = -W.at<double>(i,j)/2. - lambda * loss_d(mu_ij - mu_d, mu_d, std_d, local_std);

			// sW += W.at<double>(i,j);
			sW += exp(w);
		}

		if(sW <= 1e-10){
			gmm_sig.at<float>(r,c) = 0;
			continue;
		}

		for(int i=0; i<3; i++)
		for(int j=0; j<3; j++){
			// if((i==2 && j==0) || (i==0 && j==2) || (i==2 && j==2) || (i==0 && j==0))
			//	continue;
			double mu_ij = dpt.at<unsigned short>(r+ur+i-1, c+lc+j-1)*0.001;
			double std_ij = predict_sigma.at<float>(r+i-1, c+j-1);
			if(mu_ij <= 0.5 || std_ij <= 1e-7)
				continue;
			if(g_is_inv_dis)
				mu_ij = 1./mu_ij;
			// double w = -W.at<double>(i,j)/2. - lambda * loss(mu_ij - mu_d, mu_d, std_d, local_std);
			double w;
			if(g_is_inv_dis)
				w = -W.at<double>(i,j)/2. - lambda * loss_ind(mu_ij - mu_d, mu_d, std_d, local_std);
			else
				w = -W.at<double>(i,j)/2. - lambda * loss_d(mu_ij - mu_d, mu_d, std_d, local_std);
			w = exp(w);
			// mu_z += W.at<double>(i,j)*mu_ij/sW;
			// std_sq += W.at<double>(i,j)*(SQ(std_ij)+SQ(mu_ij))/sW;
			mu_z += w * mu_ij / sW;
			std_sq += w*(SQ(std_ij)+SQ(mu_ij))/sW;
			if(std_sq != std_sq)
				cout<<"what? mu_z: "<<mu_z<<" sW: "<<sW<<" w: "<<w<<" mu_ij: "<<mu_ij<<" std_ij: "<<std_ij<<" lambda: "<<lambda<<endl;
		}
		std_sq = std_sq - SQ(mu_z);

		if(std_sq < 0) {
			cout <<"what:  std_sq: " <<std_sq<<" r: "<<r<<" c: "<<c<<" lambda: "<<lambda <<endl;
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

	// central point's parameters
	double para[3]={0.00155816, -0.00362021, 0.00452812};
	poly predictor(para);

	for(int r=0; r<rows; r++)
	for(int c=0; c<cols; c++){
		double dis = dpt.at<unsigned short>(r+ur, c+lc)*0.001;
		double sigma;
		if(dis <= 0.5) sigma = 0;
		else{
			if(g_is_inv_dis)
				sigma = 0.0005;
			else
				sigma = predictor.y(dis);
		}
		sigma_img.at<float>(r,c) = sigma;
	}
	return sigma_img;

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
		if(g_is_inv_dis)
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
