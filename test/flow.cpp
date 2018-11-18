#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv){
	//create video capture and open appropriate camera
	cv::VideoCapture cap;
	if(argc>1){
		cap.open(std::stoi(argv[1]));
	}
	else{
		cap.open(-1);
	}

	//create a window
	cv::namedWindow("Flow", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Trajectory",cv::WINDOW_AUTOSIZE);

	cv::Mat frame, greyframe, flow, prevgrey;

	cv::Point2f flowatxy, avg_flow;

	int pcount;

	//main loop
	while(true){
		cap>>frame;
		if(frame.empty()){
			std::cout<<"Camera data lost"<<std::endl;
			break;
		}
		//Create Grey Image
		cv::cvtColor(frame, greyframe, cv::COLOR_BGR2GRAY);
		avg_flow = cv::Point2f(0,0);
		if(!prevgrey.empty()){
			cv::calcOpticalFlowFarneback(prevgrey, greyframe, flow, 0.7, 3, 20, 2, 3, 1.5, 0);
			pcount = 0;
			for (int y = 0; y < frame.rows; y += 5) {
		    	for (int x = 0; x < frame.cols; x += 5){
			// 		// get the flow from y, x position * 10 for better visibility
					flowatxy = flow.at<cv::Point2f>(y, x) * 1;
					avg_flow +=flowatxy;
			// 		// draw line at flow direction
					cv::line(frame, cv::Point(x, y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
			// 		// draw initial point
					cv::circle(frame, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1);
					pcount ++;
				}
		    }
		}
		avg_flow.x = avg_flow.x/pcount;
		avg_flow.y = avg_flow.y/pcount;

		cv::line(frame, cv::Point(frame.cols/2, frame.rows/2),
				 cv::Point(cvRound(frame.cols/2+ 20*avg_flow.x),  cvRound(frame.rows/2+ 20*avg_flow.y) ),
				 cv::Scalar(0,255,0));

		// std::cout<<"v_x = "<<avg_flow.x<<" v_y = "<<avg_flow.y<<std::endl;
		greyframe.copyTo(prevgrey);
		cv::imshow("Flow", frame);

		
		if(cv::waitKey(33)>0){break;}
	}

	//destroy window
	cv::destroyWindow("Flow");
	cv::destroyWindow("Trajectory");

}