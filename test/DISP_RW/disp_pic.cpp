#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::Mat img = cv::imread(argv[1],-1);
	if(img.empty()){return -1;}
	cv::namedWindow("Disp_picture", cv::WINDOW_AUTOSIZE);
	cv::imshow("Disp_picture", img);
	cv::waitKey(0);
	cv::destroyWindow("Disp_picture");
	return 1;
}