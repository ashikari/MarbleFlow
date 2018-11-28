#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::namedWindow("Video Display");
	cv::VideoCapture cap;
	cap.open(std::string(argv[1]));

	cv::Mat frame;
	int frame_period =33;

	if (argc > 2){
		frame_period = std::stoi(argv[2]);
	}
	
	while(true){
		cap>>frame;
		if(frame.empty()){break;}
		cv::imshow("Video Display", frame);
		if( cv::waitKey(frame_period)>0 ){break;}
	}

	cv::destroyWindow("Video Display");

	return 0;
}