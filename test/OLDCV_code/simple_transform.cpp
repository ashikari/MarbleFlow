#include <cv.h>
#include <highgui.h>

#include <iostream>


void smoother(IplImage* img, IplImage* out, int nsmooths){
	
	for(int i = 0; i<nsmooths; i++){
		cvSmooth(img, out,CV_GAUSSIAN,3,3);
	}
}

int main(int argc, char** argv){

	IplImage* img = cvLoadImage(argv[1]);
	IplImage* out = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);

	cvNamedWindow("Input_Image");
	cvNamedWindow("Output_Image");
	//display input image
	cvShowImage("Input_Image", img);
	smoother(img, out, (int)(*argv[2]) );



	//display output image
	cvShowImage("Output_Image", out);

	cvWaitKey(0);
	// cvReleaseImage(&out);
	cvDestroyWindow("Input_Image");
	cvDestroyWindow("Output_Image");

	cvReleaseImage(&img);
	cvReleaseImage(&out);
	return 0;
}