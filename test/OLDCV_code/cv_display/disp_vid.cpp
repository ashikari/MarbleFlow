#include <iostream>
#include <cv.h>
#include <highgui.h>

int main(int argc, char** argv){
	//open autosized named window
	cvNamedWindow("ex2", CV_WINDOW_AUTOSIZE);
	//allocate video object
	CvCapture* capture = cvCreateFileCapture( argv[1]);
	//allocate image object
	IplImage* frame;
	while (true){
		frame = cvQueryFrame( capture);
		if (!frame ) break;
		cvShowImage( "ex2", frame);
		char c = cvWaitKey(33);
		if (c == 27) break;
	}
	cvReleaseCapture( &capture);
	cvDestroyWindow( "ex2");


	return 0;
}
