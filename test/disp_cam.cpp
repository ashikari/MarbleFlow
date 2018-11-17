#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::VideoCapture cap;
	if (argc == 1){
		cap.open(0);
	}
	else {
		cap.open(std::stoi(argv[1])); 
	}
	cv::Mat frame;
	cv::namedWindow("Cam", cv::WINDOW_AUTOSIZE);
	while(true){
		cap>>frame;
		if(frame.empty()){
			break;
		}
		cv::imshow("Cam", frame);
		if(cv::waitKey(33)>0){
			break;
		}

	}
	cv::destroyWindow("Cam");
	return 0;
}