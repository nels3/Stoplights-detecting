#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "TrafficLightDetector.h"

#define CAM_RES_X 640
#define CAM_RES_Y 360

//class storing light variables
class light_specification {
public:
	//x coordinate where circle is
	int x;
	//y coordinate where circle is
	int y;
	//circle's radius
	int r;
	//tells if we found light
	bool found = false;
};


//class storing base variables
class base_specification {
public:
	//predictable width of light base
	int width=0;
	//predictable height of light base
	int height=0;
	//predictable corners of light base
	cv::Point upperleft;
	cv::Point downright;
	cv::Point upperright;
	cv::Point downleft;
};

//class storing variables and function to detect street lights
class LightDetector {
public:
	//thresh value
	int thresh = 254;
	//lower value to make in range with green color
	int low_green = 30;//30;
	//higher value to make in range with green color
	int up_green = 90;//90;
	//values with brightness
	int low_bright = 220;
	int up_bright = 240;
	int thresh_value = 200;

	//values use in HoughCircle
	//value that is use in Hough Circle, how close can circles be next to each other
	int param1 = 140;//200;
	//value that is use in Hough Circle, how circle can be false circle, the less value then it could be more f.e. flat
	int param2 = 25;//45;// 100;
	//value that is use in Hough Circle, how big radius can be 
	int max_radius = 0;

	//stores where is light 
	light_specification red, green;
	int green_number;
	bool green_latched;
	//stores base measurements
	base_specification base;
	//stores black rectangle
	base_specification black_base;

	std::vector<cv::Vec3f> circles_green;
	std::vector<cv::Vec3f> circles_red;

	//spliting function, depending of the output color
	void make_split_black(cv::Mat &input, cv::Mat &output, cv::Mat &hsv);
	void make_split(cv::Mat &input, cv::Mat &output, int low, int up, cv::Mat &hsv, std::vector<cv::Mat> &hsv_split);
	void make_split_green(cv::Mat &input, cv::Mat &output, cv::Mat &hsv);
	void make_split_red(cv::Mat &input, cv::Mat &output, cv::Mat &hsv);

	void prepare_image(cv::Mat &input, cv::Mat &output);
	void blur(cv::Mat &input, cv::Mat &output);
	void find_circle(cv::Mat &input, std::vector<cv::Vec3f> &circles);
	void draw_circle(cv::Mat &output, std::vector<cv::Vec3f> &circles);
	void find_red_lights(cv::Mat &input);
	void predict_street_lights();
	void draw_predicted_street_lights(cv::Mat &input);
	void find_green_lights(cv::Mat &input, std::vector<cv::Vec3f> &circles);

	void find_green_spots(cv::Mat &input);

	//methods with finding black rectangle
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i>  hierarchy;
	cv::Rect black_rectangle;
	int largest_contour_index = 0;
	void find_black_spot(cv::Mat &input, cv::Mat &output, std::vector<std::vector<cv::Point> > &contours);
	void draw_black_spot(cv::Mat &output, std::vector<std::vector<cv::Point> > &contours);

	//main methods
	void find_black_stop_lights(cv::Mat &input, cv::Mat &output);
	void find_red_stop_lights(cv::Mat &input, cv::Mat &output);
	void find_green_stop_lights(cv::Mat &input, cv::Mat &output);
	uint8_t check_status();
	uint8_t searching_for_stop_light(cv::Mat input, cv::Mat output, cv::Mat input2);
}light;

//basic split, up and down value needed
void LightDetector::make_split(cv::Mat &input, cv::Mat &output, int low, int up, cv::Mat &hsv, std::vector<cv::Mat> &hsv_split) {
	cvtColor(input, hsv, CV_BGR2HSV);
	split(hsv, hsv_split);
	inRange(hsv_split[0], low, up, output);
}

//function spliting to green image
void LightDetector::make_split_green(cv::Mat &input, cv::Mat &output, cv::Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	inRange(hsv, cv::Scalar(60, 50, 50, 0), cv::Scalar(90, 255, 255, 0), output);
}

//spliting to red image
void LightDetector::make_split_red(cv::Mat &input, cv::Mat &output, cv::Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	cv::Mat mask1(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat mask2(CAM_RES_Y, CAM_RES_X, CV_8UC1);

	inRange(hsv, cv::Scalar(0, 40, 40,0), cv::Scalar(10, 250,250,0), mask1);
	inRange(hsv, cv::Scalar(160, 40, 40,0), cv::Scalar(180, 255, 255,0), mask2);
	//inRange(hsv, cv::cv::Scalar(0, 0, 0, 0), cv::cv::Scalar(180, 255, 70, 0), output);
	output = mask1 | mask2;

}

//function spliting to black image
void LightDetector::make_split_black(cv::Mat &input, cv::Mat &output, cv::Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	inRange(hsv, cv::Scalar(0, 0, 0,0), cv::Scalar(180, 255, 70,0),output);
}

//bluring, thresholding and canny on image
void LightDetector::prepare_image(cv::Mat &input, cv::Mat &output) {
	blur(input, input);
	threshold(input, output, thresh, 255, cv::THRESH_BINARY);
	//Canny(input, output, 100, 200);
}

//finding where the biggest black spot on the image is
void LightDetector::find_black_spot(cv::Mat &input, cv::Mat &output, std::vector<std::vector<cv::Point> > &contours) {
	findContours(input, contours, light.hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image

	int max_contour_number = 0;
	int largest_area = 0;
	for (int i = 0; i< contours.size(); i++) // iterate through each contour. 
	{
		double area = contourArea(contours[i], false);  //  Find the area of contour
		if (area>largest_area) {
			largest_area = area;
			light.largest_contour_index = i;                //Store the index of largest contour
			black_rectangle = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}
	//giving values to black_base - variable that stores where stop light is
	black_base.width = black_rectangle.width;
	black_base.height = black_rectangle.height;
	black_base.upperleft.x = black_rectangle.x;
	black_base.upperleft.y = black_rectangle.y;
}

//function drawing rectangle with the biggest black spot
void LightDetector::draw_black_spot(cv::Mat &output, std::vector<std::vector<cv::Point> > &contours) {
	cv::Scalar color(255, 255, 255);
	drawContours(output, contours, light.largest_contour_index, color, CV_FILLED, 8, light.hierarchy); // Draw the largest contour using previously stored index.
	rectangle(output, black_rectangle, cv::Scalar(0, 255, 0), 1, 8, 0);
}


//making GaussianBlur
void LightDetector::blur(cv::Mat &input, cv::Mat &output) {
	GaussianBlur(input, output, cv::Size(9, 9), 2, 2);
}

//function using HoughCircle to detect circle
void LightDetector::find_circle(cv::Mat &input, std::vector<cv::Vec3f> &circles) {
	HoughCircles(input, circles, CV_HOUGH_GRADIENT, 1, input.rows / 8, light.param1, light.param2, 0, light.max_radius);
}

//function drawing specific circle
void LightDetector::draw_circle(cv::Mat &output, std::vector<cv::Vec3f> &circles) {

	for (size_t i = 0; i < circles.size(); i++)
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(output, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(output, center, radius, cv::Scalar(255, 0, 255), 3, 8, 0);
	}
}

//function detecting red street light
void LightDetector::find_red_lights(cv::Mat &input) {
	blur(input, input);
	//prepare_image(input, input);
	find_circle(input, circles_red);
	//when there is only one red circle
	if (circles_red.size() == 1 && circles_red[0][2] != 0) {
		red.x = circles_red[0][0];
		red.y = circles_red[0][1];
		red.r = circles_red[0][2];
		std::cout << "middle: x = " << light.red.x << " y = " << light.red.y << " radius = " << light.red.r << std::endl;
		red.found = true;
	}
	//when there are more than one red circle
	else if (circles_red.size() != 1) {
		//function to check if there aren't the same circle //to check on examples
		for (size_t i = 1; i < circles_red.size(); i++) {
			for (size_t j = 0; j < i; j++) {
				if (abs(circles_red[j][0] - circles_red[i][0]) < 10 && abs(circles_red[j][1] - circles_red[i][1]) < 10 && circles_red[j][2] != 0) {
					std::cout << "found more the same red circle" << std::endl;
					red.found = true;
				}
				else {
					std::cout << "found more red circles" << std::endl;
					red.found = false;
				}
			}

		}
		//dopisac ze jesli dwa a blisko siebie to by traktowal jak jeden okrag!!!
	}
	//when error occurs or there isn't red circle
	else {
		std::cout << "Error in hough lines - no circles" << std::endl;
	}

}

//function detecting green street light
void LightDetector::find_green_lights(cv::Mat &input, std::vector<cv::Vec3f> &circles) {
	blur(input, input);
	find_circle(input, circles_green);
	//when there is only one red circle
	if (circles.size() >0) {
		green.found = true;
		std::cout << "Green cricle found" << std::endl;
	}
	//when error occurs or there isn't red circle
	else {
		std::cout << "No green circles" << std::endl;
	}
}

//function detecting number of green pixels
void LightDetector::find_green_spots(cv::Mat &input) {	
	if (green_latched == false) {
		green_number = cv::countNonZero(input);
		green_latched = true;
	}
	else {
		if (cv::countNonZero(input) - green_number > 600) {
			green.found = true;
			//std::cout << "Green light - go!!!" << std::endl;
		}
		else {
			green.found = false;
			//std::cout << "No green light" << std::endl;
		}
	}
	//std::cout << countNonZero(input) << std::endl;
}

//function predicting base of street lights, making smaller spot to detect green light
void LightDetector::predict_street_lights() {
	int width = (int)(2.5*red.r);
	int height = (int)(0.5*red.r);
	//variables were chosen by trials
	base.upperleft.x = red.x - width;
	base.upperleft.y = red.y + 2*height;
	base.downright.x = red.x + width;
	base.downright.y = red.y + 11 * height;
	base.width = base.downright.x - base.upperleft.x;
	base.height = base.downright.y - base.upperleft.y;
}

//drawing base of street lights
void LightDetector::draw_predicted_street_lights(cv::Mat &input) {
	rectangle(input, base.upperleft, base.downright, cv::Scalar(0, 255, 0), 2, 8, 0);
}

//detecting black area
void LightDetector::find_black_stop_lights(cv::Mat &input, cv::Mat &output) {
	cv::Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat hsv(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	make_split_black(input, img, hsv);
	prepare_image(img, img);
	find_black_spot(img, output, contours);
	draw_black_spot(output, contours);
}

//detecting red lights
void LightDetector::find_red_stop_lights(cv::Mat &input, cv::Mat &output) {
	cv::Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat hsv(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	//making ROI where black stop lights are
	cv::Mat roi = input(black_rectangle);
	//spliting to red color
	make_split_red(roi, img, hsv);

	//finding red cirles
	find_red_lights(img);
	draw_circle(roi, circles_red);
	if (red.found == true) {
		predict_street_lights();
		draw_predicted_street_lights(output);
	}
	else {
		std::cout << "Still looking for red circle" << std::endl;
	}
}

//detecting red lights
void LightDetector::find_green_stop_lights(cv::Mat &input, cv::Mat &output) {
	cv::Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat hsv(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Rect Rec(base.upperleft.x + black_base.upperleft.x, base.upperleft.y + black_base.upperleft.y, base.width, base.height);
	//making roi for green light
	cv::Mat roi = input(Rec);
	make_split_green(roi, output, hsv);
	//find_green_lights(img, circles_green);
	find_green_spots(output);
}

//checking where in code we are
uint8_t LightDetector::check_status() {
	uint8_t status = 0;
	//we are searching for base
	if (black_base.width == 0)
		status = 1;
	//we are looking for red circle
	else if (red.found == false){
		status = 2;
		light.param2 = light.param2 - 1;
	}
	//we are waiting for green change in light
	else if (red.found == true and green.found == false) 
		status = 3;
	//we have green light
	else if (green.found == true) 
		status = 4;
	return status;
}

//main method
uint8_t LightDetector::searching_for_stop_light(cv::Mat input, cv::Mat output, cv::Mat input2) {
	cv::Mat display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	switch (check_status()) {
	case 1:
		std::cout << "Searching for base" << std::endl;
		//finding where black stop light base is:
		light.find_black_stop_lights(input, output);
		break;
	case 2:
		light.find_red_stop_lights(input, display);
		break;
	case 3:
		light.find_green_stop_lights(input, display);
		break;
	case 4:
		//we found green light
		break;
	}
	
	return check_status();
}

int main()
{
	cv::Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Mat display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	
	cv::namedWindow("frame", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("display", CV_WINDOW_AUTOSIZE);
	
	//createTrackbar("param1", "img", &light.param1, 300, NULL);
	//createTrackbar("param2", "img", &light.param2, 300, NULL);
	//createTrackbar("thresh", "img", &light.thresh, 300, NULL);
	
	VideoCapture capture;
	capture.open(0, CAP_V4L2);
	if (!capture.isOpened()) {
		std::cout << "error with camera" << std::endl;
	}
	else {
		capture.set(CAP_PROP_FRAME_WIDTH, 640);
		capture.set(CAP_PROP_FRAME_HEIGHT, 360);
		capture.set(CAP_PROP_CONVERT_RGB, false);
		cout << "MODE" << capture.get(CAP_PROP_MODE) << endl;
		cout << "RGB" << capture.get(CAP_PROP_CONVERT_RGB) << endl;
	}

	while (true) {
		capture >> frame;
		uint8_t status = light.searching_for_stop_light(frame, display, frame2);
		if (status == 4) {
			std::cout << "GO!!!" << std::endl;
		}
		else {
			std::cout << "WAIT!!!" << std::endl;
		}
		cv::waitKey(100);
	
		
		imshow("display", display);
		imshow("frame", frame);
	}

	capture.release();

	waitKey();
	return 0;
}


