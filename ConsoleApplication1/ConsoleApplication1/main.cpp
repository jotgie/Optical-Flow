

#include "stdafx.h"
#include <math.h>
#include "mMinWindow.h"
#include "BGS.h"

using namespace std;
using namespace cv;

int main(void)
{
	Size winSize(15, 15), subPixWinSize(15, 15);
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.3);
	Mat mFrame0, mFrame1, mFrame2, mFrame3, frame;
	Mat* result1;
	Mat* result2;
	VideoCapture cap;
	cv::Rect win1, win2;
	cap.open("banan.avi");

	if (!cap.isOpened()) {
		cout << "Cannot open the video file" << endl;
		return -1;
	}

	
	cap >> frame;

	win1 = cv::Rect(0, 0, frame.cols, frame.rows);
	//win2 = cv::Rect(1150, 500, 150, 300);
	mMinWindow mMinFrame0 = mMinWindow(win1, winSize, subPixWinSize, termcrit);
	//mMinWindow mMinFrame1 = mMinWindow(win2, winSize, subPixWinSize, termcrit);
	BGS bgsFrame0 = BGS(win1, 30, 20, 0, 149, 160);
	//BGS bgsFrame1 = BGS(win2, 30, 20, 0, 149, 120);
	/*cv::Mat mFrame_Wrapper(
	cv::Size(mMinFrame0.getWidth() * 2 + 50,
	mMinFrame0.getHeigth()),
	CV_8UC3);

	cv::Mat mFrame_Wrapper_Counturs(
	cv::Size(mMinFrame0.getWidth() * 2 + 50,
	mMinFrame0.getHeigth()),
	CV_8UC3);
	*/
	cv::Mat mFrame_Wrapper_Mask(
		cv::Size(mMinFrame0.getWidth() * 2 + 50,
			mMinFrame0.getHeigth()),
		CV_8UC1);

	while (true)
	{
	

		cap >> frame;

		if (frame.empty()) {
			fprintf(stderr, "End of video");
			return -1;
		}

		
		mFrame0 = mMinFrame0.drawVectors(frame);
		/*if (!mFrame0.empty()) 
			mFrame0.copyTo(mFrame_Wrapper(
				cv::Rect(
					0,
					0,
					mMinFrame0.getWidth(),
					mMinFrame0.getHeigth())));
		

		mFrame1 = mMinFrame1.drawVectors(frame);
		if (!mFrame1.empty()) 
			mFrame1.copyTo(mFrame_Wrapper(
				cv::Rect(
					mMinFrame0.getWidth() + 50,
					0,
					mMinFrame1.getWidth(),
					mMinFrame1.getHeigth())));
		*/
		result1 = bgsFrame0.drawSquare(frame, mMinFrame0.getResultVector());
		mFrame2 = result1[0];
		if (!mFrame2.empty()) 
		mFrame2.copyTo(mFrame_Wrapper_Mask(
			cv::Rect(
				0,
				0,
				mFrame2.cols,
				mFrame2.rows)));
		
		/*
		result2 = bgsFrame1.drawSquare(frame, mMinFrame0.getResultVector());
		mFrame3 = result2[0];
		if (!mFrame3.empty()) 
			mFrame3.copyTo(mFrame_Wrapper_Mask(
				cv::Rect(
					mMinFrame0.getWidth() + 50,
					0,
					mFrame3.cols,
					mFrame3.rows)));
		*/

		mFrame2 = result1[1];
		/*if (!mFrame2.empty()) 
			mFrame2.copyTo(mFrame_Wrapper_Counturs(
				cv::Rect(
					0,
					0,
					mFrame2.cols,
					mFrame2.rows)));
		

		mFrame3 = result2[1];
		if (!mFrame3.empty()) 
			mFrame3.copyTo(mFrame_Wrapper_Counturs(
				cv::Rect(
					mMinFrame0.getWidth() + 50,
					0,
					mFrame3.cols,
					mFrame3.rows)));
		*/
	
		
		imshow("Optical Flow", mFrame2);
		imshow("Mask", mFrame_Wrapper_Mask);
		//imshow("Counturs", mFrame_Wrapper_Counturs);
		int key_pressed = waitKey(1);
		if (key_pressed == 'q') break;
			
	}

	return 0;
}