#include <opencv2/opencv.hpp>
#include <iostream>


// void Dilation( int, void* )
// {
//   int dilation_type = 0;
//   if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
//   else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
//   else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }
//   cv::Mat element = cv::getStructuringElement( dilation_type,
//                        cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//                        cv::Point( dilation_size, dilation_size ) );
//   cv::dilate( src, dilation_dst, element );
// }

int main(int argc, char** argv){
	//open camera

	cv::VideoCapture cap;
	if (argc==1){
		cap.open(0);
	}
	else{
		cap.open(std::stoi(argv[1]));
	}

	//Create Named Window
	cv::namedWindow("Transformed", cv::WINDOW_AUTOSIZE);

	cv::Mat frame;

	int dilation_size = 2;
	cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                       cv::Point( dilation_size, dilation_size ) );
	while(true){
		cap>>frame;
		cv::erode(frame, frame, element,cv::Point(-1,-1), 5);
		cv::dilate(frame, frame, element,cv::Point(-1,-1), 5);
		if (frame.empty() or cv::waitKey(33)>0){
			break;
		}


		cv::imshow("Transformed",frame);

	}

	cv::destroyWindow("Transformed");
	return 0;
}