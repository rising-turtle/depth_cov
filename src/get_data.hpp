/*

	Dec. 16, 2019, He Zhang, hzhang8@vcu.edu 

	different ways to compute data from a Mat 

*/

#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <algorithm>
#include <vector>

template<typename T>
T getMean(const cv::Mat& M, int r, int c, int w)
{
	int rr = M.rows; 
	int cc = M.cols; 

	T mu = 0; 
	double sum = 0; 
	int cnt = 0; 

	for(int y=r-w; y<=r+w; y++)
	for(int x=c-w; x<=c+w; x++){
		if(x< 0 || x>= cc || y<0 || y>=rr)
			continue; 

		T v = M.at<T>(y,x); 
		if(v > T(0)){
			sum += v; 
			cnt++; 
		}
	}
	if(cnt > 0){
		mu = sum/cnt; 
	}
	return T(mu); 
}

template<typename T>
T getMedian(const cv::Mat& M, int r, int c, int w)
{
	int rr = M.rows; 
	int cc = M.cols; 

	T me = 0; 
	std::vector<T> vs;

	for(int y=r-w; y<=r+w; y++)
	for(int x=c-w; x<=c+w; x++){
		if(x< 0 || x>= cc || y<0 || y>=rr)
			continue; 

		T v = M.at<T>(y,x); 
		if(v > T(0)){
			vs.push_back(v);
		}
	}
	if(vs.size() > 0){

		if(vs.size() == 1)
			me = vs[0];
		else{
			std::sort(vs.begin(), vs.end()); 
			if(vs.size() & 0x01){
				// old 
				me = vs[vs.size()/2]; 
			}else{
				me = (vs[vs.size()/2-1]+vs[vs.size()/2])/2;
			}
		}
	}
	return T(me); 
}
