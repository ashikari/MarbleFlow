#include <opencv2/opencv.hpp>
#include<iostream>
#include "movie_maker.hpp"
#include "flow.hpp"
#include <librealsense2/rs.hpp>

int main(int argc, char**argv){
	//open video feed
	// cv::VideoCapture cap;
	// if(argc==1){
	// 	cap.open(0);
	// }
	// else{
	// 	cap.open(std::stoi(argv[1]));
	// } 
	// if(!cap.isOpened()){
	// 	std::cerr<<"Couldn't open capture."<<std::endl;
	// 	return -1;
	// }


	cv::Mat frame, greyframe, prevgrey;


	//code to run the script with intel realsense cameras
	//Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;
    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }



	//draw the initial image
	cv::Mat img =  boarder();

	double fps  = 90;
 	cv::VideoWriter writer("outcpp.avi",CV_FOURCC('M','J','P','G'),fps, img.size()); 
	cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Cam", CV_WINDOW_AUTOSIZE);

	float dt = 1/fps;
	cv::Point2f flow(0,0);
	cv::Point2f pt(1280/2,720/2);
    while(true){

    	//Get each frame
   		frames = pipe.wait_for_frames();
    	rs2::frame color_frame = frames.get_color_frame();
    	// Creating OpenCV Matrix from a color image
    	frame = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

    	cv::cvtColor(frame, greyframe, cv::COLOR_BGR2GRAY);
    	cv::GaussianBlur(greyframe,greyframe,cv::Size(5,5),7,7);
    	if (~prevgrey.empty()){
    		avg_flow(greyframe,prevgrey,flow);
    		cv::circle(img, pt, 2, cv::Scalar(0,0,255), CV_FILLED);
    		pt = pt + flow*7.5/640.0*17;

    		std::cout<<pt<<" "<< flow<<std::endl;
    	}
    	writer<<img;

    	cv::line(frame, cv::Point(frame.cols/2, frame.rows/2),
				 cv::Point(cvRound(frame.cols/2+ 20*flow.x),  cvRound(frame.rows/2 + 20*flow.y) ),
				 cv::Scalar(0,255,0), 5);

    	greyframe.copyTo(prevgrey);
    	cv::imshow("Display", img);
    	cv::imshow("Cam",frame);
    	if( cv::waitKey( 1 )>0){
    		break;
    	}
    }
	
	cv::destroyWindow("Display");
	cv::destroyWindow("Cam");

}