//not so simple transformation

#include <cv.h>
#include <highgui.h>

IplImage* doPyrDown(IplImage* in, int filter = CV_GAUSSIAN_5x5){
	assert(in->width%2==0 &&in->height%2 ==0);
	IplImage* out = cvCreateImage(cvSize(in->width/2, in->height/2), in->depth, in->nChannels);
	cvPyrDown(in, out);
	return(out);
}


int main(int argc, char** argv){
	cvNamedWindow("Ex5-in");
	cvNamedWindow("Ex5-out");
	IplImage* input = cvLoadImage(argv[1]);
	IplImage* output = doPyrDown(input);

	cvShowImage("Ex5-in", input);
	cvShowImage("Ex5-out",output);

	cvReleaseImage(&input);
	cvReleaseImage(&output);
	//Destory Window
	cvWaitKey(0);
	cvDestroyWindow("Ex5-in");
	cvDestroyWindow("Ex5-out");

	return 0;
}