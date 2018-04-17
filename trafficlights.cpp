
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "TrafficLightDetector.h"
using namespace cv;
using namespace std;

#define CAM_RES_X 640
#define CAM_RES_Y 360

RNG rng(12345);

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
	int width;
	//predictable height of light base
	int height;
	//predictable corners of light base
	Point upperleft;
	Point downright;
	Point upperright;
	Point downleft;
};

//class storing variables and function to detect street lights
class LightDetector {
public:
	//lower value to make in range with red color
	int low_red = 160;//0
	//higher value to make in range with red color
	int up_red = 190;//10
	//lower value to make in range with green color
	int low_green = 65;//30;
	//higher value to make in range with green color
	int up_green = 100;//90;
	//values with brightness
	int low_bright = 220;
	int up_bright = 240;
	int thresh_value = 200;

	//values use in HoughCircle
	//value that is use in Hough Circle, how close can circles be next to each other
	int param1 = 140;//200;
	//value that is use in Hough Circle, how circle can be false circle, the less value then it could be more f.e. flat
	int param2 = 45;// 100;
	//value that is use in Hough Circle, how big radius can be 
	int max_radius = 0;

	//stores where is light 
	light_specification red, green;
	//stores base measurements
	base_specification base;

	vector<Vec3f> circles_green;
	vector<Vec3f> circles_green_sym;
	vector<Vec3f> circles_red;

	void make_split(Mat &input, Mat &output, int low, int up, Mat &hsv, vector<Mat> &hsv_split);
	void blur(Mat &input, Mat &output);
	void find_circle(Mat &input, vector<Vec3f> &circles);
	void draw_circle(Mat &output, vector<Vec3f> &circles);
	void find_red_lights(Mat &input);
	void predict_street_lights();
	void draw_predicted_street_lights(Mat &input);
	void find_green_lights(Mat &input, vector<Vec3f> &circles);
}light;

void LightDetector::make_split(Mat &input, Mat &output, int low, int up, Mat &hsv, vector<Mat> &hsv_split) {
	cvtColor(input, hsv, CV_BGR2HSV);
	split(hsv, hsv_split);
	inRange(hsv_split[0], low, up, output);
}

//making GaussianBlur
void LightDetector::blur(Mat &input, Mat &output) {
	GaussianBlur(input,output, Size(9, 9), 2, 2);
}

//function using HoughCircle to detect circle
void LightDetector::find_circle(Mat &input, vector<Vec3f> &circles) {
	HoughCircles(input, circles, CV_HOUGH_GRADIENT, 1, input.rows / 8, light.param1, light.param2, 0, light.max_radius);
}

//function drawing specific circle
void LightDetector::draw_circle(Mat &output, vector<Vec3f> &circles){

	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle(output, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		// circle outline
		circle(output, center, radius, Scalar(255, 0, 255), 3, 8, 0);
	}
}

//function detecting red street light
void LightDetector::find_red_lights(Mat &input) {
	light.blur(input,input);
	light.find_circle(input, light.circles_red);
	if (light.circles_red.size() != 1) {
		//function to check if there aren't the same circle //to check on examples
		for (size_t i = 1; i < light.circles_red.size(); i++) {
			for (size_t j = 0; j < i; i++) {
				if (abs(light.circles_red[j][0] - light.circles_red[i][0]) < 10 && abs(light.circles_red[j][1] - light.circles_red[i][1]) < 10) {
					cout << "found more the same red circle" << endl;
					light.red.found = true;
				}
				else {
					cout << "found more red circles" << endl;
					light.red.found = false;
				}
			}

		}
		//dopisac ze jesli dwa a blisko siebie to by traktowal jak jeden okrag!!!
	}
	else {
		light.red.x = light.circles_red[0][0];
		light.red.y = light.circles_red[0][1];
		light.red.r = light.circles_red[0][2];
		cout << "middle: x = " << light.red.x << " y = " << light.red.y << " radius = " << light.red.r << endl;
		light.red.found = true;
	}
}

//function detecting green street light
void LightDetector::find_green_lights(Mat &input, vector<Vec3f> &circles) {
	light.blur(input, input);
	light.find_circle(input, light.circles_green);
	//cout << light.circles_green.size() << endl;
	if (circles.size() > 0) {
		cout << "Found green"<<endl;
		light.green.found = true;
	}
	else {
		cout << "No green light"<<endl;
	}
}


//function predicting base of street lights
void LightDetector::predict_street_lights() {
	int width = (int)(2.5*light.red.r);
	int height = (int)(0.5*light.red.r);
	//variables were chosen by trials
	light.base.upperleft.x = light.red.x - width;
	light.base.upperleft.y = light.red.y + height;
	light.base.downright.x = light.red.x + width;
	light.base.downright.y = light.red.y + 8*height;
	light.base.width = light.base.downright.x - light.base.upperleft.x;
	light.base.height = light.base.downright.y - light.base.upperleft.y;
}

//drawing base of street lights
void LightDetector::draw_predicted_street_lights(Mat &input) {
	rectangle(input, light.base.upperleft, light.base.downright, Scalar(0,255,0), 2, 8, 0);
}

int main()
{
	cv::Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	//symulacja, okno z zielonym swiatlem
	cv::Mat frame2(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Mat img_green_sym(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat green_display_sym(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::namedWindow("display_green_sym", CV_WINDOW_AUTOSIZE);


	cv::Mat hsv(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Mat img_red(CAM_RES_Y, CAM_RES_X, CV_8UC1);

	cv::Mat img_green(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat img_roi(CAM_RES_Y, CAM_RES_X, CV_8UC1);

	cv::Mat red_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Mat green_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::namedWindow("img", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("display_red", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("display_green", CV_WINDOW_AUTOSIZE);
	
	vector<Mat> hsv_split;
	/*
	createTrackbar("lred","img", &light.low_red, 255, NULL);
	createTrackbar("ured", "img", &light.up_red, 255, NULL);
	createTrackbar("lgreen", "img", &light.low_green, 255, NULL);
	createTrackbar("ugreen", "img", &light.up_green, 255, NULL);
	createTrackbar("lbright", "img", &light.low_bright, 255, NULL);
	createTrackbar("ubright", "img", &light.up_bright, 255, NULL);
	createTrackbar("thresh", "img", &light.thresh_value, 255, NULL);
	*/

	//createTrackbar("param1", "img", &light.param1, 300, NULL);
	//createTrackbar("param2", "img", &light.param2, 300, NULL);

	
	const std::string file_name = "test1.jpg";
	const std::string file_name_sym = "test2.jpg"; //symulacja
	frame = cv::imread(file_name);
	frame2 = cv::imread(file_name_sym);
	if (!frame.data) {
		std::cout << "Nie odnalezionu pliku " << file_name;
		return -1;
	}
	if (!frame2.data) {
		std::cout << "Nie odnalezionu pliku " << file_name_sym;
		return -1;
	}

	frame.copyTo(red_display);

	while (waitKey(20) != 27){

		if (light.red.found == false) {
			light.make_split(frame, img_red, light.low_red, light.up_red, hsv, hsv_split);
			light.find_red_lights(img_red);
			light.draw_circle(red_display, light.circles_red);
			light.predict_street_lights();
			light.draw_predicted_street_lights(red_display);
		}
		else {
			//Rectangle used to define region of interest
			Rect Rec(light.base.upperleft.x, light.base.upperleft.y, light.base.width, light.base.height);
			//if (light.green.found == false) {
				//starting symulation
				cout << "No green light_start"<<endl;
				Mat roi_sym = frame(Rec);			
				light.make_split(roi_sym, img_green_sym, light.low_green, light.up_green, hsv, hsv_split);
				light.find_green_lights(img_green_sym, light.circles_green_sym);
				cout << "No green light_end" << endl;
				//stoping symulation

				cout << "Green light_start"<<endl;
				//making roi
				Mat roi = frame2(Rec);
				light.make_split(roi, img_green, light.low_green, light.up_green, hsv, hsv_split);
				light.find_green_lights(img_green,light.circles_green);
				light.draw_circle(green_display, light.circles_green);
				cout << "Green light_end" << endl;
			//}
			cv::waitKey(600);
	
		}
		
		imshow("display_red", red_display);

		if (light.red.found == true) {
			
			imshow("display_green_sym", img_green_sym);
			imshow("display_green", img_green);

		}


	
		cv::waitKey(200);
	}

	cv::waitKey(0);
	return(0);
}
