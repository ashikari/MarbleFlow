int rgb2hsv(int red, int grn, int blu);
int getKeypointHue(cv::KeyPoint keyPt, cv::Mat frame);
cv::Vec3b hsv2rgb(int hue);


//Create Blob Tracker
cv::Ptr<cv::SimpleBlobDetector> create_tracker(){
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 1;
    params.maxThreshold = 255;
    params.filterByArea = true;
    params.minArea = 500;
    params.minDistBetweenBlobs = 30;
    // params.filterByCircularity = true;
    // params.minCircularity = 0.1;
    // params.filterByConvexity = true;
    // params.minConvexity = 0.6;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;
    return cv::SimpleBlobDetector::create(params);
}

int dilation_size = 1;
cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                    cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                    cv::Point( dilation_size, dilation_size ) );



void preprocess(cv::Mat& frame, cv::Mat& grayFrame){
        cv::GaussianBlur(frame, grayFrame, cv::Size(5,5), 3, 3);
        cv::dilate(grayFrame, grayFrame, element,cv::Point(-1,-1), 10);
        cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5,5), 3, 3);
        cv::erode(grayFrame, grayFrame, element, cv::Point(-1,-1), 10);
        cv::GaussianBlur(grayFrame, grayFrame, cv::Size(5,5), 3, 3);
        cv::cvtColor(grayFrame, grayFrame, cv::COLOR_BGR2GRAY);
        // cv::bitwise_not(grayFrame, grayFrame);
}


void calibrate(cv::VideoCapture& cap, cv::Ptr<cv::SimpleBlobDetector>& detector, const int& NUM_BALLS,  int keypointHues[], int hueBins[] ){
    cv::Mat frame, grayFrame;
    std::vector<cv::KeyPoint> keypoints;

    

    while(true){
        cap>>frame;
        preprocess(frame,grayFrame);
        detector->detect(grayFrame, keypoints);             // Detect blobs.

        // Calibrate ball colors
        if (keypoints.size()==NUM_BALLS) {
            for (int k=0;k<NUM_BALLS;k++) {
                keypointHues[k] = getKeypointHue(keypoints[k], frame);
            }
            std::sort(keypointHues, keypointHues+NUM_BALLS);
            for (int b=0;b<NUM_BALLS-1;b++) {
                hueBins[b] = (keypointHues[b]+keypointHues[b+1])/2;
            }
            hueBins[NUM_BALLS-1] = 360;
            break;
        }


        cv::imshow("Calibration", grayFrame);
        if(frame.empty()){
            break;
        }
        // cv::imshow("Cam", frame);
        if(cv::waitKey(33)>0){
            break;
        }

    
    }
}



//Utility Functions

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