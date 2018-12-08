#include <opencv2/opencv.hpp>
#include <iostream>

const int NUM_BALLS = 2;

static int rgb2hsv(int red, int grn, int blu);
static int getKeypointHue(cv::KeyPoint keyPt, cv::Mat frame);

int main(int argc, char** argv){
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
	int hueBins[NUM_BALLS+2];
	while(true){
		cap>>frame;
		
		cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

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

		// Detect blobs.
		detector->detect(grayFrame, keypoints);

		// Calibrate ball colors
		if (!calibrated && keypoints.size()==NUM_BALLS) {
			for (int k=0;k<NUM_BALLS;k++) {
				keypointHues[k] = getKeypointHue(keypoints[k], frame);
			}
			std::sort(keypointHues, keypointHues+sizeof(keypointHues)/sizeof(keypointHues[0]));
			hueBins[0] = 0;
			for (int b=0;b<NUM_BALLS-1;b++) {
				hueBins[b+1] = (keypointHues[b]+keypointHues[b+1])/2;
			}
			hueBins[NUM_BALLS+1] = 255;
		}
		std::cout<<hueBins<<std::endl;

		for (int k=0;k<keypoints.size();k++) {
			cv::KeyPoint keyPt = keypoints[k];
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
			int hue = rgb2hsv(red,grn,blu);
			cv::Vec3b val;
			if (hue<27) {
				val = cv::Vec3b(0,0,255);
			} else if (hue<115) {
				val = cv::Vec3b(42,42,165);
			} else if (hue<220) {
				val = cv::Vec3b(255,0,0);
			} else {
				val = cv::Vec3b(255,0,255);
			}
			for (int x=-r;x<r;x++) {
				for (int y=-std::sqrt(r*r-x*x);y<std::sqrt(r*r-x*x);y++) {
					frame.at<cv::Vec3b>(center.y+y,center.x+x) = val;
				}
			}

			//std::cout<<k<<","<<val<<",";
		}
		//std::cout<<std::endl;

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