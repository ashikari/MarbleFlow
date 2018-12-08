#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	cv::Mat color, grey, canny;

	color = cv::imread(argv[1]);

	cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Grey", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Canny", cv::WINDOW_AUTOSIZE);

	cv::imshow("Color", color);

	cv::cvtColor(color, grey, cv::COLOR_BGR2GRAY); //what is the color_bgr2gray mean?

	cv::imshow("Grey", grey);

	cv::Canny(grey, canny, 10, 100, 3, true); //what do the canny parameters mean?

	cv::imshow("Canny", canny);

	cv::waitKey(0);

	cv::destroyWindow("Color");
	cv::destroyWindow("Grey");
	cv::destroyWindow("Canny");

}