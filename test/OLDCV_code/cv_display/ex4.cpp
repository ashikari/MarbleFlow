//simple smoothing transformation

#include <cv.h>
#include <highgui.h>

int main(int argc, char** argv){
	//Created named windows for input and output
	cvNamedWindow("Ex4-in");
	cvNamedWindow("Ex4-out");

	//open image
	IplImage* image = cvLoadImage(argv[1]);

	cvShowImage("Ex4-in", image);

	//allocate memory for output image
	IplImage* out = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);

	//smooth image
	cvSmooth(image,out, CV_GAUSSIAN, 9, 9);
	cvShowImage("Ex4-out", out);

	//delete image memory
	cvReleaseImage(&out);

	cvWaitKey(0);
	cvDestroyWindow("Ex4-in");
	cvDestroyWindow("Ex4-out");


	return 0;

}