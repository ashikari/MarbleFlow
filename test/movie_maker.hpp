#pragma once

#include <opencv2/opencv.hpp>
#include<iostream>

int inch2Pixel(float l){
	float scale = 17;//pixels per inch
	return round(scale*l);
}

cv::Mat boarder(){
	// To create an image
    // CV_8UC3 depicts : (3 channels,8 bit image depth
    // Height  = 500 pixels, Width = 1000 pixels
    //set image scale
    cv::Mat img(720, 1280, CV_8UC3, cv::Scalar(255,255, 255));


    //Draw Border
    //color
    cv::Scalar grey(192,192,192);
    //corners of board

    //lengths of the stage
    //scale to pixels
    int inch = inch2Pixel(1);
    int s_width = inch2Pixel(32);
    int s_height = inch2Pixel(36);
    cv::Point2i center(1280/2, 720/2);
    cv::Point2i pt1 = center - cv::Point2i(s_width/2+inch, s_height/2+inch);
    cv::Point2i pt2 = pt1 + cv::Point2i(s_width + 2*inch, inch);
    cv::Point2i pt3 = pt2 + cv::Point2i(-inch, s_height+inch);
    cv::Point2i pt4 = pt3 + cv::Point2i(-s_width-inch, -inch ); 
    cv::Point2i pt5 = pt1 + cv::Point2i(inch, inch);

    //drawing
    //center
    // cv::circle(img, center, inch2Pixel(0.99), grey, CV_FILLED);
    //Boundaries
    cv::rectangle(img, pt1, pt2, grey, CV_FILLED);
    cv::rectangle(img, pt2, pt3, grey, CV_FILLED);
    cv::rectangle(img, pt3, pt4, grey, CV_FILLED);
    cv::rectangle(img, pt4, pt5, grey, CV_FILLED);

    return img;
   }


