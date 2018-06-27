#include <iostream>
#include <stdio.h>
#include <stddef.h>

#include "opencv2/opencv.hpp"
#include "ueye.h"

#include "ids.h"

IDS_PARAMETERS ids_camera;
HIDS hCam = 0;

int main()
{
	ids_camera.initialize_camera(&hCam);
	ids_camera.setting_auto_params(&hCam);

	cvNamedWindow("frame", 1);
	cvNamedWindow("ids", 1);
	cv::createTrackbar("Pixel", "ids", &ids_camera.pixelclock_slider, 80, NULL);
	cv::createTrackbar("Exposure", "ids", &ids_camera.exposure_slider, 500, NULL);
	cv::createTrackbar("FPS", "ids", &ids_camera.fps_slider, 90, NULL);
	cv::createTrackbar("Master", "ids", &ids_camera.Master_GAIN_Factor, 100, NULL);
	cv::createTrackbar("Green", "ids", &ids_camera.Green_GAIN_Factor, 100, NULL);
	cv::createTrackbar("Red", "ids", &ids_camera.Red_GAIN_Factor, 100, NULL);
	cv::createTrackbar("Blue", "ids", &ids_camera.Blue_GAIN_Factor, 100, NULL);

	while (true) {
		
		cv::Mat ids_image (480, 752, CV_8UC3);
		ids_camera.get_frame(&hCam,752,480,ids_image);
		ids_camera.change_params(&hCam);
		//is_GetFramesPerSecond(hCam, &ids_camera.NEWFPS);
		std::cout << "FPS: "<<ids_camera.NEWFPS<< std::endl;
		
		cv::imshow("frame", ids_image);

		
		if ((int)cv::waitKey(10) >= 0) {
			break;
		}
	}
	
	// Release the camera 
	is_ExitCamera(hCam);

	return 0;
}
