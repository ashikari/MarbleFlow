#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>

#define VERBOSE 1

const int NUM_BALLS = 2;

static int rgb2hsv(int red, int grn, int blu);
static int getKeypointHue(cv::KeyPoint keyPt, cv::Mat frame);
static cv::Vec3b hsv2rgb(int hue);

int main(int argc, char** argv){
	// ROS Setup
	ros::init(argc, argv, "camera");
	ros::NodeHandle n;
	ros::Publisher ball_pub = n.advertise<std_msgs::Int32MultiArray>("/ball_talk", 1000);
	ros::Rate loop_rate(10);

	// Declare VideoCapture object and open camera.
	cv::VideoCapture cap;
	if (argc == 1){
		cap.open(0);
	}
	else {
		cap.open(std::stoi(argv[1])); 
	}

	cv::Mat frame, grayFrame;
	cv::namedWindow("Cam", cv::WINDOW_AUTOSIZE);
	bool calibrated = false;
	int keypointHues[NUM_BALLS];
	int hueBins[NUM_BALLS];
	// Set up blob detection.
	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 10;
	params.maxThreshold = 200;
	params.filterByArea = true;
	params.minArea = 1500;
	params.filterByCircularity = true;
	params.minCircularity = 0.1;
	params.filterByConvexity = true;
	params.minConvexity = 0.87;
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;
	std::vector<cv::KeyPoint> keypoints;
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);   
	
	while(!calibrated){
		cap>>frame;
		cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
		detector->detect(grayFrame, keypoints);				// Detect blobs.

		// Calibrate ball colors
		if (!calibrated && keypoints.size()==NUM_BALLS) {
			for (int k=0;k<NUM_BALLS;k++) {
				keypointHues[k] = getKeypointHue(keypoints[k], frame);
			}
			std::sort(keypointHues, keypointHues+sizeof(keypointHues)/sizeof(keypointHues[0]));
			for (int b=0;b<NUM_BALLS-1;b++) {
				hueBins[b] = (keypointHues[b]+keypointHues[b+1])/2;
			}
			hueBins[NUM_BALLS-1] = 360;
			calibrated = true;
		}

		if(frame.empty()){
			break;
		}
		cv::imshow("Cam", frame);
		if(cv::waitKey(33)>0){
			break;
		}
	}

	#ifdef VERBOSE
	std::cout<<"Calibrated ball colors. Bins are: 0-";
	for (int bin : hueBins) {
		std::cout<<bin<<"-";
	}
	std::cout<<std::endl;
	std::cout<<"Ball hues are: ";
	for (int hue : keypointHues) {
		std::cout<<hue<<",";
	}
	std::cout<<std::endl;

	#endif
	
	while (true) {
		cap>>frame;
		cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
		detector->detect(grayFrame, keypoints);				// Detect blobs.

		for (int k=0;k<keypoints.size();k++) {
			cv::KeyPoint keyPt = keypoints[k];
			cv::Point2f center = keyPt.pt;
			int r = keyPt.size/2;

			// Determine average hue of each ball
			int blu=0,grn=0,red=0,pxls=0;
			for (int x=-r;x<r;x++) {
				for (int y=-std::sqrt(r*r-x*x);y<std::sqrt(r*r-x*x);y++) {
					blu+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[0];
					grn+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[1];
					red+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[2];
					pxls++;
				}
			}
			blu/=pxls;grn/=pxls;red/=pxls;
			int hue = rgb2hsv(red,grn,blu);
			
			// Classify object
			int ballColor;
			for (int b=0;b<NUM_BALLS;b++) {
				if (hue<=hueBins[b]) {
					ballColor = b;
					break;
				}
			}

			// Draw classification over object in image
			cv::Vec3b ballColorVec = hsv2rgb(keypointHues[ballColor]);
			for (int x=-r;x<r;x++) {
				for (int y=-std::sqrt(r*r-x*x);y<std::sqrt(r*r-x*x);y++) {
					frame.at<cv::Vec3b>(center.y+y,center.x+x) = ballColorVec;
				}
			}
		}
		
		// Calculate Error from Ball 0
		cv::KeyPoint keyPt = keypoints[0];
		cv::Point2f center = keyPt.pt;
		int r = keyPt.size/2;
		int err_x = center.x-frame.rows/2;
		int err_y = center.y-frame.cols/2;
		std_msgs::Int32MultiArray msg;
	    std::vector<int> data;
	    data.push_back(err_x);
	    data.push_back(err_y);
	    msg.data = data;
	    ball_pub.publish(msg);
	    ros::spinOnce();

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

int getKeypointHue(cv::KeyPoint keyPt, cv::Mat frame) {
	cv::Point2f center = keyPt.pt;
	int r = keyPt.size/2;

	int blu=0,grn=0,red=0,pxls=0;
	for (int x=-r;x<r;x++) {
		for (int y=-std::sqrt(r*r-x*x);y<std::sqrt(r*r-x*x);y++) {
			blu+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[0];
			grn+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[1];
			red+=frame.at<cv::Vec3b>(center.y+y,center.x+x)[2];
			pxls++;
		}
	}
	blu/=pxls;grn/=pxls;red/=pxls;
	return rgb2hsv(red,grn,blu);
}

int rgb2hsv(int red, int grn, int blu)
{
    double      min, max, delta;
    double      hue, sat, value;

    min = red < grn ? red : grn;
    min = min  < blu ? min  : blu;

    max = red > grn ? red : grn;
    max = max  > blu ? max  : blu;

    value = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        sat = 0;
        hue = 0; // undefined, maybe nan?
        return hue;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        sat = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        sat = 0.0;
        hue = -1000;                            // its now undefined
        return hue;
    }
    if( red >= max )                           // > is bogus, just keeps compilor happy
        hue = ( grn - blu ) / delta;        // between yellow & magenta
    else
    if( grn >= max )
        hue = 2.0 + ( blu - red ) / delta;  // between cyan & yellow
    else
        hue = 4.0 + ( red - grn ) / delta;  // between magenta & cyan

    hue *= 60.0;                              // degrees

    if( hue < 0.0 )
        hue += 360.0;

    return hue;
}

cv::Vec3b hsv2rgb(int hue)
{
    double      hh, p, q, t, ff;
    long        i;
    cv::Vec3b   out;

    hh = hue;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = 0.0;
    q = 255*(1.0 - ff);
    t = 255*(1.0 - (1.0 - ff));

    switch(i) {
	    case 0:
	        out[2] = 255;
	        out[1] = t;
	        out[0] = p;
	        break;
	    case 1:
	        out[2] = q;
	        out[1] = 255;
	        out[0] = p;
	        break;
	    case 2:
	        out[2] = p;
	        out[1] = 255;
	        out[0] = t;
	        break;

	    case 3:
	        out[2] = p;
	        out[1] = q;
	        out[0] = 255;
	        break;
	    case 4:
	        out[2] = t;
	        out[1] = p;
	        out[0] = 255;
	        break;
	    case 5:
	    default:
	        out[2] = 255;
	        out[1] = p;
	        out[0] = q;
	        break;
    }
    return out;
}