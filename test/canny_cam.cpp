#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::VideoCapture cap;

	if(argc==1){
		cap.open(-1);
	}
	else{
		cap.open(std::stoi(argv[1]));
	}

	cv::Mat frame;

	cv::namedWindow("Canny Cam", cv::WINDOW_AUTOSIZE);


	while(true){
		cap>>frame;
		if(frame.empty()){break;}
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::Canny(frame, frame, 10, 100, 3 ,true);
		cv::imshow("Canny Cam", frame);
		if(cv::waitKey(33)>0){break;}

	}

	cv::destroyWindow("Canny Cam");
	return 0;
}