#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

void avg_flow(cv::Mat& greyframe, cv::Mat& prevgrey, cv::Point2f& avg_flow){
		avg_flow = cv::Point2f(0,0);
		cv::Mat flow;
		float pcount = 0;
		if(!prevgrey.empty()){
			cv::calcOpticalFlowFarneback(prevgrey, greyframe, flow, 0.4, 1, 12, 2, 8, 1.2, 0);
			for (int y = 0; y < greyframe.rows; y += 2) {
		    	for (int x = 0; x < greyframe.cols; x += 2){
					avg_flow +=flow.at<cv::Point2f>(y, x);
					pcount ++;
				}
		    }
		}
		else{pcount = 1;}
		avg_flow.x = avg_flow.x/pcount;
		avg_flow.y = avg_flow.y/pcount;
	}