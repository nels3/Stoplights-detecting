/************************************************************************
Project:
Selfie, automonous car
Authot:
Kornelia Lukojc
Descripiton:
Main file that works with StopLightDetector class
*************************************************************************/


#include "stoplights.h"

#include "pthread.h"

#define FRAME_WIDTH 640 //1280
#define FRAME_HEIGHT 360 //720

StopLightDetector stoplight;

void *thread(void *ptr)
{
	int type = (int)ptr;
	fprintf(stderr, "Thread - %d\n", type);
	return  ptr;
}

int main()
{
	cv::Mat frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC4);
	cv::Mat old_frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	cv::Mat diffrence(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	

	cv::namedWindow("Input frame", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Bright difference", CV_WINDOW_AUTOSIZE);

	//cv::createTrackbar("down", "frame", &light.down, 50, NULL);
		
	cv::VideoCapture capture;
	capture.open(0);// , cv::CAP_V4L2);

	if (!capture.isOpened()) {
		std::cout << "error with camera" << std::endl;
	}
	else {
		//capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		//capture.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
		//capture.set(cv::CAP_PROP_CONVERT_RGB, true);
	}

	capture >> frame;
	stoplight.prepare_first_image(frame, old_frame, stoplight.roi_number);
	cv::waitKey(100);

	while (true) {
		capture >> frame;
	
		//stoplight.test_roi(frame, frame);
		stoplight.find_start(frame, diffrence, old_frame, stoplight.roi_number);

		if (stoplight.start_finding == true) {
			imshow("Bright difference", diffrence);
			imshow("Input frame", frame);
		}
		
		cv::waitKey(100);
	}
	capture.release();
	cv::waitKey();
	return 0;
}