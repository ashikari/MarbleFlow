#include <opencv2/opencv.hpp>
#include<iostream>
#include "movie_maker.hpp"
#include "flow.hpp"

int main(int argc, char**argv){
	//open video feed
	cv::VideoCapture cap;
	if(argc==1){
		cap.open(0);
	}
	else{
		cap.open(std::stoi(argv[1]));
	} 
	if(!cap.isOpened()){
		std::cerr<<"Couldn't open capture."<<std::endl;
		return -1;
	}
	cv::Mat frame, greyframe, prevgrey;

	//draw the initial image
	cv::Mat img =  boarder();

	double fps  = 30;
 	cv::VideoWriter writer("outcpp.avi",CV_FOURCC('M','J','P','G'),fps, img.size()); 
	cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Cam", CV_WINDOW_AUTOSIZE);

	float dt = 1/fps;
	cv::Point2f flow(0,0);
	cv::Point2f pt(1280/2,720/2);
    while(true){
    	cap>>frame;
    	cv::cvtColor(frame, greyframe, cv::COLOR_BGR2GRAY);
    	cv::GaussianBlur(greyframe,greyframe,cv::Size(5,5),5,5);
    	if (~prevgrey.empty()){
    		avg_flow(greyframe,prevgrey,flow);
    		cv::circle(img, pt, 2, cv::Scalar(0,0,255), CV_FILLED);
    		pt = pt - flow*0.5;
    		std::cout<<pt<<" "<< flow<<std::endl;
    	}
    	writer<<img;


    	cv::line(frame, cv::Point(frame.cols/2, frame.rows/2),
				 cv::Point(cvRound(frame.cols/2+ 20*flow.x),  cvRound(frame.rows/2 + 20*flow.y) ),
				 cv::Scalar(0,255,0), 5);

    	greyframe.copyTo(prevgrey);
    	cv::imshow("Display", img);
    	cv::imshow("Cam",frame);
    	if( cv::waitKey( round( dt*1000 ) )>0){
    		break;
    	}
    }
	
	cv::destroyWindow("Display");
	cv::destroyWindow("Cam");

}