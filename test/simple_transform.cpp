#include <cv.h>
#include <highgui.h>

#include <iostream>


void smoother(IplImage* img, int nsmooths){
	cvNamedWindow("Input_Image");
	cvNamedWindow("Output_Image");
	//display input image
	cvShowImage("Input_Image", img);

	// IplImage* out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
	// cvSmooth(img, out, CV_GAUSSIAN,3,3);

	for(int i = 0; i<nsmooths; i++){
		cvSmooth(img, img,CV_GAUSSIAN,3,3);
	}

	//display output image
	cvShowImage("Output_Image", img);

	cvWaitKey(0);
	cvReleaseImage(&img);
	// cvReleaseImage(&out);
	cvDestroyWindow("Input_Image");
	cvDestroyWindow("Output_Image");
}


int main(int argc, char** argv){
	IplImage* img = cvLoadImage(argv[1]);
	smoother(img, (int)(*argv[2]) );


	cvReleaseImage(&img);
	return 0;
}