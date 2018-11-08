#include <iostream>
#include <cv.h>
#include "highgui.h"

int main(int argc, char** argv){
	IplImage* img = cvLoadImage(argv[1]);
	cvNamedWindow("ex1", CV_WINDOW_AUTOSIZE);
	cvShowImage("ex1", img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow("ex1");
}
