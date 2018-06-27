/************************************************************************
Project:
Selfie, automonous car
Authot:
Kornelia Lukojc
Descripiton:
Main file that works with StopLightDetector class
*************************************************************************/
#include <pthread.h>
#include <stdio.h>

pthread_mutex_t frame_mutex;

/* this function is run by the second thread */
void *camera_frame(void *x_void_ptr){
	
	int *x_ptr = (int *)x_void_ptr;
	pthread_mutex_lock(&frame_mutex);
	//get frame from camera
	while(++(*x_ptr) < 150);
	pthread_mutex_unlock(&frame_mutex);
	printf("Camera finished\n");
	return NULL;
}

void *find_lines(void *x_void_ptr){
	
	//using frame image

	int *x_ptr = (int *)x_void_ptr;
	while(++(*x_ptr) < 450);
	printf("Find lines finished\n");
	return NULL;
}

void *find_cones(void *x_void_ptr){
	//using frame image

	int *x_ptr = (int *)x_void_ptr;
	while(++(*x_ptr) < 500);
	printf("Find cones increment finished\n");
	return NULL;
}

int main()
{

	int x = 0, y = 0, z=0;

	//values
	printf("x: %d, y: %d, z: %d\n", x, y,z);

	//creating pthread
	pthread_t frame_thread, lines_thread,cones_thread;
	//pthread_mutex_t frame_mutex;
	pthread_mutex_init(&frame_mutex,NULL);

	/* create a second thread which executes inc_x(&x) */
	if(pthread_create(&frame_thread, NULL, camera_frame, &x)) {
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}
	if(pthread_create(&lines_thread, NULL, find_lines, &y)) {
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}
	if(pthread_create(&cones_thread, NULL, find_cones, &z)) {
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}
	
	
	pthread_join(frame_thread, NULL);
	pthread_join(lines_thread, NULL);
	pthread_join(cones_thread, NULL);

	/* show the results - x is now 100 thanks to the second thread */
	printf("x: %d, y: %d, z: %d\n", x, y,z);

	return 0;

}

/*

//#include "stoplights.h"
#include<stdio.h>
#include <pthread.h>

#define FRAME_WIDTH 640 //1280
#define FRAME_HEIGHT 360 //720

//StopLightDetector stoplight;

void *thread(void *ptr)
{
	//int type = (int)ptr;
	fprintf(stderr, "Thread\n");
	return  ptr;
}

int main()
{
	while(true){
	// create the thread objs
    	pthread_t thread1, thread2;
    	int thr = 1;
    	int thr2 = 2;
    	// start the threads
    	pthread_create(&thread1, NULL, *thread, (void *) thr);
    	pthread_create(&thread2, NULL, *thread, (void *) thr2);
    	// wait for threads to finish
    	pthread_join(thread1,NULL);
	pthread_join(thread2,NULL);
	}
	//cv::Mat frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC4);
	//cv::Mat old_frame(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	//cv::Mat diffrence(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	

	//cv::namedWindow("Input frame", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("Bright difference", CV_WINDOW_AUTOSIZE);
	//cv::createTrackbar("down", "frame", &light.down, 50, NULL);
	//cv::VideoCapture capture;
	//capture.open(0);// , cv::CAP_V4L2);
	//if (!capture.isOpened()) {
	//	std::cout << "error with camera" << std::endl;
	//}
	//else {
		//capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		//capture.set(cv::CAP_PROP_FRAME_HEIGHT, 360);
		//capture.set(cv::CAP_PROP_CONVERT_RGB, true);
	//}

	//capture >> frame;
	//stoplight.prepare_first_image(frame, old_frame, stoplight.roi_number);
	//cv::waitKey(100);

	//while (true) {
		//capture >> frame;
	
		//stoplight.test_roi(frame, frame);
		//stoplight.find_start(frame, diffrence, old_frame, stoplight.roi_number);

		//if (stoplight.start_finding == true) {
		//	imshow("Bright difference", diffrence);
		//	imshow("Input frame", frame);
		//}
		
		//cv::waitKey(100);
	//}
	//capture.release();
	//cv::waitKey();
	return 0;
}
*/
