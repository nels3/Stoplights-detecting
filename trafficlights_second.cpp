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
	Point upperleft;
	Point downright;
	Point upperright;
	Point downleft;
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

	vector<Vec3f> circles_green;
	vector<Vec3f> circles_green_sym;
	vector<Vec3f> circles_red;

	//spliting function, depending of the output color
	void make_split_black(Mat &input, Mat &output, Mat &hsv);
	void make_split(Mat &input, Mat &output, int low, int up, Mat &hsv, vector<Mat> &hsv_split);
	void make_split_green(Mat &input, Mat &output, Mat &hsv);
	void make_split_red(Mat &input, Mat &output, Mat &hsv);

	void prepare_image(Mat &input, Mat &output);
	void blur(Mat &input, Mat &output);
	void find_circle(Mat &input, vector<Vec3f> &circles);
	void draw_circle(Mat &output, vector<Vec3f> &circles);
	void find_red_lights(Mat &input);
	void predict_street_lights();
	void draw_predicted_street_lights(Mat &input);
	void find_green_lights(Mat &input, vector<Vec3f> &circles);

	void find_green_spots(Mat &input);

	//function with finding black rectangle
	vector<vector<Point> > contours;
	vector<Vec4i>  hierarchy;
	Rect black_rectangle;
	int largest_contour_index = 0;
	void find_black_spot(Mat &input, Mat &output, vector<vector<Point> > &contours);
	void draw_black_spot(Mat &output, vector<vector<Point> > &contours);

}light;

//basic split, up and down value needed
void LightDetector::make_split(Mat &input, Mat &output, int low, int up, Mat &hsv, vector<Mat> &hsv_split) {
	cvtColor(input, hsv, CV_BGR2HSV);
	split(hsv, hsv_split);
	inRange(hsv_split[0], low, up, output);
}

//function spliting to green image
void LightDetector::make_split_green(Mat &input, Mat &output, Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	inRange(hsv, cv::Scalar(60, 50, 50, 0), cv::Scalar(90, 255, 255, 0), output);
}

//spliting to red image
void LightDetector::make_split_red(Mat &input, Mat &output, Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	cv::Mat mask1(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat mask2(CAM_RES_Y, CAM_RES_X, CV_8UC1);

	inRange(hsv, cv::Scalar(0, 40, 40,0), cv::Scalar(10, 250,250,0), mask1);
	inRange(hsv, cv::Scalar(160, 40, 40,0), cv::Scalar(180, 255, 255,0), mask2);
	//inRange(hsv, cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 70, 0), output);
	output = mask1 | mask2;

}

//function spliting to black image
void LightDetector::make_split_black(Mat &input, Mat &output, Mat &hsv) {
	cvtColor(input, hsv, CV_BGR2HSV);
	inRange(hsv, cv::Scalar(0, 0, 0,0), cv::Scalar(180, 255, 70,0),output);
}

//bluring, thresholding and canny on image
void LightDetector::prepare_image(Mat &input, Mat &output) {
	blur(input, input);
	threshold(input, output, thresh, 255, THRESH_BINARY);
	//Canny(input, output, 100, 200);
}

//finding where the biggest black spot on the image is
void LightDetector::find_black_spot(Mat &input, Mat &output, vector<vector<Point> > &contours) {
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
void LightDetector::draw_black_spot(Mat &output, vector<vector<Point> > &contours) {
	Scalar color(255, 255, 255);
	drawContours(output, contours, light.largest_contour_index, color, CV_FILLED, 8, light.hierarchy); // Draw the largest contour using previously stored index.
	rectangle(output, black_rectangle, Scalar(0, 255, 0), 1, 8, 0);
}


//making GaussianBlur
void LightDetector::blur(Mat &input, Mat &output) {
	GaussianBlur(input, output, Size(9, 9), 2, 2);
}

//function using HoughCircle to detect circle
void LightDetector::find_circle(Mat &input, vector<Vec3f> &circles) {
	HoughCircles(input, circles, CV_HOUGH_GRADIENT, 1, input.rows / 8, light.param1, light.param2, 0, light.max_radius);
}

//function drawing specific circle
void LightDetector::draw_circle(Mat &output, vector<Vec3f> &circles) {

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
	blur(input, input);
	//prepare_image(input, input);
	find_circle(input, circles_red);
	//when there is only one red circle
	if (circles_red.size() == 1 && circles_red[0][2] != 0) {
		red.x = circles_red[0][0];
		red.y = circles_red[0][1];
		red.r = circles_red[0][2];
		cout << "middle: x = " << light.red.x << " y = " << light.red.y << " radius = " << light.red.r << endl;
		red.found = true;
	}
	//when there are more than one red circle
	else if (circles_red.size() != 1) {
		//function to check if there aren't the same circle //to check on examples
		for (size_t i = 1; i < circles_red.size(); i++) {
			for (size_t j = 0; j < i; j++) {
				if (abs(circles_red[j][0] - circles_red[i][0]) < 10 && abs(circles_red[j][1] - circles_red[i][1]) < 10 && circles_red[j][2] != 0) {
					cout << "found more the same red circle" << endl;
					red.found = true;
				}
				else {
					cout << "found more red circles" << endl;
					red.found = false;
				}
			}

		}
		//dopisac ze jesli dwa a blisko siebie to by traktowal jak jeden okrag!!!
	}
	//when error occurs or there isn't red circle
	else {
		cout << "Error in hough lines - no circles" << endl;
	}

}

//function detecting green street light
void LightDetector::find_green_lights(Mat &input, vector<Vec3f> &circles) {
	blur(input, input);
	find_circle(input, circles_green);
	//when there is only one red circle
	if (circles_green.size() >0) {
		green.found = true;
		cout << "Green cricle found" << endl;
	}
	//when error occurs or there isn't red circle
	else {
		cout << "No green circles" << endl;
	}
}

//function detecting number of green pixels
void LightDetector::find_green_spots(Mat &input) {	
	if (green_latched == false) {
		green_number = cv::countNonZero(input);
		green_latched = true;
	}
	else {
		if (cv::countNonZero(input) - green_number > 2000) {
			green.found = true;
			cout << "Green light - go!!!" << endl;
		}
		else {
			cout << "No green light" << endl;
		}
	}
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
void LightDetector::draw_predicted_street_lights(Mat &input) {
	rectangle(input, base.upperleft, base.downright, Scalar(0, 255, 0), 2, 8, 0);
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
	cv::Mat img_black(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat img_green(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat img_roi(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	cv::Mat roi_frame(CAM_RES_Y, CAM_RES_X, CV_8UC4);

	cv::Mat red_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	//cv::Mat black_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::Mat green_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	cv::namedWindow("img", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("display_red", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("display_green", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Display", CV_WINDOW_AUTOSIZE);
	vector<Mat> hsv_split;
	
	//createTrackbar("param1", "img", &light.param1, 300, NULL);
	//createTrackbar("param2", "img", &light.param2, 300, NULL);
	//createTrackbar("thresh", "img", &light.thresh, 300, NULL);
	
	const std::string file_name = "test2_r.jpg";
	const std::string file_name_sym = "test2_g.jpg"; //symulacja
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

	//frame.copyTo(red_display);

	while (waitKey(20) != 27) {
		if (light.black_base.width == 0) {
			//finding where black stop light base is:
			cv::Mat black_display(CAM_RES_Y, CAM_RES_X, CV_8UC4);  //new Mat to clear what was before
			light.make_split_black(frame, img_black, hsv);
			light.prepare_image(img_black, img_black);
			light.find_black_spot(img_black, black_display, light.contours);
			light.draw_black_spot(black_display, light.contours);
			cout << "Searching for base" << endl;
		}

		else {

			if (light.red.found == false) {
				//making ROI where black stop lights are
				roi_frame = frame(light.black_rectangle);
				//spliting to red color
				light.make_split_red(roi_frame, img_red, hsv);
	
				//finding red cirles
				light.find_red_lights(img_red);
				light.draw_circle(roi_frame, light.circles_red);
				if (light.red.found == true) {
					light.predict_street_lights();
					light.draw_predicted_street_lights(red_display);
				}
				else {
					cout << "still looking for" << endl;
				}
			}
			
			else {
				//Rectangle used to define region of interest
				Rect Rec(light.base.upperleft.x+light.black_base.upperleft.x, light.base.upperleft.y + light.black_base.upperleft.y, light.base.width, light.base.height);
				
				//if (light.green.found == false) {
				//starting symulation
				cout << "No green light_start" << endl;
				//making roi for green light
				Mat roi_sym = frame(Rec);
				light.make_split_green(roi_sym, img_green_sym, hsv);
				light.find_green_lights(img_green_sym, light.circles_green_sym);
				light.find_green_spots(img_green_sym);
				cout << "No green light_end" << endl;
				//stoping symulation
				
				cout << "Green light_start" << endl;
				//making roi for green light
				Mat roi = frame2(Rec);
				light.make_split_green(roi, img_green, hsv);
				light.find_green_lights(img_green, light.circles_green);
				light.find_green_spots(img_green);
				cout << "Green light_end" << endl;
				//}
				
				cv::waitKey(600);

			}
			
		}
		//imshow("display_red", roi_frame);
		//imshow("display_green_sym", roi_frame);
		//imshow("display_red", img_red);
		if (light.red.found == true) {
			imshow("display_green_sym", img_green_sym);
			imshow("display_green", img_green);
		}
		cv::waitKey(200);
	}

	cv::waitKey(0);
	return(0);
}

