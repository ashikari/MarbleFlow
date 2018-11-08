
#include <iostream>
#include <cv.h>
#include <highgui.h>

//define global variables
int g_slider_pos = 0;
CvCapture* g_capture = NULL;

//callback function for setting the slider position
void onTrackbarSlide(int pos){
	cvSetCaptureProperty(g_capture, CV_CAP_PROP_POS_FRAMES, pos);
}

int main(int argc, char ** argv){
	//create autosized named window
	cvNamedWindow("Ex3", CV_WINDOW_AUTOSIZE);

	//create file capture
	g_capture = cvCreateFileCapture(argv[1]);
	int nFrames = cvGetCaptureProperty(g_capture, CV_CAP_PROP_FRAME_COUNT);
	if (nFrames!=0){
		cvCreateTrackbar("Position", "Ex3", &g_slider_pos, nFrames, onTrackbarSlide);
	}
	//allocate image
	IplImage* frame;
	//while loop over file
	while(1){
		frame = cvQueryFrame(g_capture);
		//show new frame in window
		if (!frame) break;
		cvShowImage("Ex3", frame);
		char c  = cvWaitKey(15);
		if (c == 27) break; // hitting escape will break the loop
		cvSetTrackbarPos("Position", "Ex3", cvGetCaptureProperty(g_capture, CV_CAP_PROP_POS_FRAMES));
	}
	//release file
	cvReleaseCapture(&g_capture);
	//destroy file capture
	cvDestroyWindow("Ex3");
	return 0;
}