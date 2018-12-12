#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include "blob_tracker.hpp"

int main(int argc, char** argv){
	// Setup ROS
	ros::init(argc, argv, "ballz_tracker");
	ros::NodeHandle ballz_tracker;
	ros::Publisher error_pub = ballz_tracker.advertise<std_msgs::Int32MultiArray>("ball_error", 1);
	std_msgs::Int32MultiArray msg;
	ros::Rate loop_rate(10);

	// Open camera capture
	cv::VideoCapture cap;
	cap.open(1);

	cv::Mat frame, grayFrame;
	cap>>frame;

	// Create Windows
	cv::namedWindow("Calibration", cv::WINDOW_AUTOSIZE);

	// Create tracker
	cv::Ptr<cv::SimpleBlobDetector> detector = create_tracker();

	// bool calibrated = false;
	int NUM_BALLS = 4;
	int keypointHues[NUM_BALLS];
	int hueBins[NUM_BALLS];

	// Perform calibration
	calibrate(cap, detector, NUM_BALLS, keypointHues, hueBins);

	cv::destroyWindow("Calibration");
	cv::namedWindow("Cam", cv::WINDOW_AUTOSIZE);

	std::vector<cv::KeyPoint> keypoints;


	cv::Point2f error;
	cv::Point2f img_center(frame.cols/2.0, frame.rows/2.0);	

	int dilation_size = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );

	while (true) {
		cap>>frame;
		if(frame.empty()){
			break;
		}
		std::cout<<frame.size()<<std::endl;

		preprocess(frame,grayFrame);
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
			cv::circle(frame, center,r,cv::Scalar(ballColorVec),CV_FILLED);
			cv::arrowedLine(frame, center, img_center, cv::Scalar(ballColorVec), 5);

			error = img_center - center;
		}

		error = img_center - keypoints[0].pt;
		//publish the ros message and spin
		msg.data.clear()
		msg.data[0] = error.x;
		msg.data[1] = error.y;
		error_pub.publish(msg);
     	ros::spinOnce();

		cv::imshow("Cam", frame);
		if(cv::waitKey(33)>0){
			break;
		}
	}

	cv::destroyWindow("Cam");
	return 0;
}