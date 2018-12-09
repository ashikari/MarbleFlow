#include <opencv2/opencv.hpp>
#include<iostream>

int inch2Pixel(float l){

	float scale = 1;//pixels per inch
	return round(scale*l);
}


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

	//get camera feed parameters
	// double fps = cap.get(CV_CAP_PROP_FPS); // giving -1 a whole bunch for some reason
	double fps  = 60;

	cv::Size sz(
		(int)cap.get(CV_CAP_PROP_FRAME_WIDTH),
		(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT)
		);

	
 	cv::VideoWriter writer("outcpp.avi",CV_FOURCC('M','J','P','G'),fps, sz); 
	cv::namedWindow("Camera", CV_WINDOW_AUTOSIZE);

	// To create an image
    // CV_8UC3 depicts : (3 channels,8 bit image depth
    // Height  = 500 pixels, Width = 1000 pixels
    //set image scale
    cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(255,255, 255));

    cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);

    //Draw Border
    //color
    cv::Scalar grey(192,192,192);
    //corners of board

    //lengths of the stage
    //scale to pixels
    int inch = inch2Pixel(10.0);
    int s_width = inch2Pixel(100);
    int s_height = inch2Pixel(100);
    cv::Point2i center(1280/2, 720/2);
    cv::Point2i pt1 = center - cv::Point2i(s_width/2+inch, s_height/2+inch);
    cv::Point2i pt2 = pt1 + cv::Point2i(s_width + 2*inch, inch);
    cv::Point2i pt3 = pt2 + cv::Point2i(-inch, s_height+inch);
    cv::Point2i pt4 = pt3 + cv::Point2i(-s_width-inch, -inch ); 
    cv::Point2i pt5 = pt1 + cv::Point2i(inch, inch);

    //drawing
    //center
    cv::circle(img, center, 10, grey, CV_FILLED);
    //Boundaries
    cv::rectangle(img, pt1, pt2, grey, CV_FILLED);
    cv::rectangle(img, pt2, pt3, grey, CV_FILLED);
    cv::rectangle(img, pt3, pt4, grey, CV_FILLED);
    cv::rectangle(img, pt4, pt5, grey, CV_FILLED);

    cv::imshow("Display", img);

    cv::Mat frame;
    while(true){
    	cap>>frame;
    	writer<<frame;
    	cv::imshow("Camera", frame);
    	if( cv::waitKey( round( 1/fps*1000 ) )>0){
    		break;
    	}
    }

    cap.release();
    cv::destroyWindow("Display");
    cv::destroyWindow("Camera");

	return 0;
}