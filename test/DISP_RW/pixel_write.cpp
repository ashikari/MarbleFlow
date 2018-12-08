#include <opencv2/opencv.hpp>
#include <iostream>


int main(int argc, char** argv){
	if(argc!=4){
		std::cout<<"script requires 3 arguments: file_name, x, y"<<std::endl;
		return 0;
	}


	cv::Mat img = cv::imread(argv[1]);
	int x = std::stoi(argv[2]), y = std::stoi(argv[3]);
	cv::Vec3b intensity = img.at<cv::Vec3b>(y,x);

	uint8_t blue = intensity[0];
	uint8_t green = intensity[1];
	uint8_t red = intensity[2];

	std::cout<<"red at ("<<x<<","<<y<<") is "<<(int)red<<std::endl;
	std::cout<<"green at ("<<x<<","<<y<<") is "<<(int)green<<std::endl;
	std::cout<<"blue at ("<<x<<","<<y<<") is "<< (int)blue <<std::endl;

	cv::namedWindow("Pixels", cv::WINDOW_AUTOSIZE);
	cv::Vec3b val(255,255,255);
	val[0] = 0;
	val[1] = 0;

	for(int i = 0; i<std::min(img.rows, img.cols)-std::max(x,y); i++){
		img.at<cv::Vec3b>(y+1+i, x+1+i) = val;
		img.at<cv::Vec3b>(y-1+i, x-1+i) = val;
		img.at<cv::Vec3b>(y+1+i, x-1+i) = val;
		img.at<cv::Vec3b>(y-1+i, x+1+i) = val;
	}

	cv::imshow("Pixels", img);
	cv::waitKey(0);
	cv::destroyWindow("Pixels");

	

}