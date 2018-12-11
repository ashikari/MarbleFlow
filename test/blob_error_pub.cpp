#include <opencv2/opencv.hpp>
#include <iostream>
#include "blob_tracker.hpp"


// #include "ros/ros.h"
// #include "std_msgs/Int32MultiArray"


int main(int argc, char** argv){

	//setup ROS
	// ros::init(argc, argv, "ball_error");
	// ros::NodeHandle ball_error;
	// ros::Publisher error_pub = n.advertise<std_msgs::Int32MultiArray>("ball_error", 1);
	// std_msgs::Int32MultiArray msg;
	// ros::Rate loop_rate(10);



	//open camera capture
	cv::VideoCapture cap;
	cap.open(1);

	cv::Mat frame, grayFrame;
	cap>>frame;


	//Create Windows:
	cv::namedWindow("Cam", cv::WINDOW_AUTOSIZE);

	//Create tracker
	cv::Ptr<cv::SimpleBlobDetector> detector = create_tracker();

	// bool calibrated = false;
	int NUM_BALLS = 1;
	int keypointHues[NUM_BALLS];
	int hueBins[NUM_BALLS];

	//perform calibration
	calibrate(cap, detector, NUM_BALLS, keypointHues, hueBins);

	
	std::vector<cv::KeyPoint> keypoints;


	cv::Point2f error;
	cv::Point2f img_center(frame.cols/2.0, frame.rows/2.0);
	//perform blob tracking
	

	int dilation_size = 1;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );

	while (true) {
		cap>>frame;
		if(frame.empty()){
			break;
		}

		// for (int g=0;g<10;g++){
		// 	// cv::GaussianBlur(frame, frame, cv::Size(5,5), 3, 3);
		// }
		cv::erode(frame, frame, element,cv::Point(-1,-1), 10);
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
			cv::circle(frame, center,r,cv::Scalar(ballColorVec),CV_FILLED);
			cv::arrowedLine(frame, center, img_center, cv::Scalar(ballColorVec), 5);


			error = img_center - center;

			//publish the ros message and spin
			// msg.data.clear()
			// msg.data[0] = error.x;
			// msg.data[1] = error.y;
			// error_pub.publish(msg);
	     	// ros::spinOnce();
		}





		cv::imshow("Cam", frame);
		if(cv::waitKey(33)>0){
			break;
		}

	}

	cv::destroyWindow("Cam");
	return 0;




}