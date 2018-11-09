#include <iostream>
#include <cv.h>
#include <highgui.h>

int main(int argc, char** argv){
	//open autosized named window
	cvNamedWindow("camera", CV_WINDOW_AUTOSIZE);
	//allocate video object

	std::cout<<argv[1]<<std::endl;
	std::cout<< std::stoi(argv[1]) <<std::endl;
	CvCapture* capture = cvCreateCameraCapture( std::stoi(argv[1]) );
	//allocate image object
	IplImage* frame;
	while (true){
		frame = cvQueryFrame( capture);
		if (!frame ) break;
		cvShowImage( "camera", frame);
		char c = cvWaitKey(33); //escape key?
		if (c == 27) break;
	}
	cvReleaseCapture( &capture);
	cvDestroyWindow( "ex2");


	return 0;
}
