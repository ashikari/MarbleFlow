#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::Mat in = cv::imread(argv[1], -1); // what does the "-1" do?
	cv::Mat out;

	cv::namedWindow("in", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("out",cv::WINDOW_AUTOSIZE );

	cv::imshow("in", in);

	cv::GaussianBlur(in, out, cv::Size(5,5), 3, 3); // what do the parameters in the function mean?

	for(int i = 0; i<std::stoi(argv[2])-1; i++){
		cv::GaussianBlur(out, out, cv::Size(5,5), 3, 3);
	}


	cv::imshow("out", out);
	cv::waitKey(0);

	cv::destroyWindow("in");
	cv::destroyWindow("out");
}