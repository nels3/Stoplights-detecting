#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "opencv2/core/utility.hpp"
#include "opencv2/imgcodecs.hpp"
#include "math.h"

#include <stdexcept>
#include "opencv_utils.h"

using namespace cv;
using namespace std;
//#include "line_detection.h"

#define CAM_RES_X 640
#define CAM_RES_Y 360

using namespace std;
using namespace cv;


class filt_mask {
public:
	int startX = 0;
	int startY = 240;

	int maskx1 = 0;
	int masky1 = CAM_RES_Y;
	int maskx2 = 0;
	int masky2 = 200;

	int maskx3 = 100;
	int masky3 = 60;
	int maskx4 = CAM_RES_X - maskx3;
	int masky4 = masky3;

	int maskx5 = CAM_RES_X;
	int masky5 = masky2;
	int maskx6 = CAM_RES_X;
	int masky6 = CAM_RES_Y;
	
	Point trapez[6] = {
		Point(maskx1, masky1),
		Point(maskx2, masky2),
		Point(maskx3, masky3),
		Point(maskx4, masky4),
		Point(maskx5, masky5),
		Point(maskx6, masky6)
	};

};
filt_mask mask_point;
RNG rng(12345);

struct Lane {
	vector<Point> line;
	uint64_t frame;
	int priority;
};

class car_data{
public:
	double threshvalue = 40;
	int roiX = 320;
	int roiY = 60;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Lane> lanes;

};

car_data value;

class car_function{
public:
	void convert_to_gray(Mat &input, Mat &output) {
		cvtColor(input, output, CV_BGR2GRAY);
	}
	void make_blur(Mat &input, Mat &output) {
		blur(input, output, Size(3, 3));
	}
	void mask(Mat &input, Mat &output) {
		Mat mask = Mat::zeros(cv::Size(640, 360), CV_8UC1);
		fillConvexPoly(mask, mask_point.trapez, 6, cv::Scalar(255, 0, 0));
		namedWindow("maska", CV_WINDOW_AUTOSIZE);
		imshow("maska", mask);
		bitwise_and(input, mask, output);
	}
	void make_canny(Mat &input, Mat &output) {
		Canny(input, output, value.threshvalue, value.threshvalue *3 , 3);
	}

	//Function using approxPolyDP
	void contours(Mat &input, Mat& output) {
		findContours(input, value.contours, value.hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		//delete smaller than 40 contours
		for (vector<vector<Point> >::iterator filt = value.contours.begin(); filt != value.contours.end();) {
			if (filt->size()<40) 
				filt = value.contours.erase(filt);
			else 
				++filt;
		}

		for (int i = 0; i < value.contours.size(); i++) {
			drawContours(output, value.contours, i, Scalar(255, 255, 0), 2, 8, value.hierarchy, 0, Point());
			approxPolyDP(Mat(value.contours[i]), value.contours[i], 10, true);		
			for (unsigned int j = 0; j<value.contours[i].size(); j++) {
				circle(output, value.contours[i][j], 3, Scalar(0, 255, 255), FILLED, LINE_AA);
				if (j>0)
					line(output, Point(value.contours[i][j-1]), Point(value.contours[i][j]), Scalar(0, 0, 255), 2);
			}
		}
	}

	//Function using fit line
	void fit_line(Mat &input, Mat &output) {
		Vec4f lines;
		float * ratio;
		findContours(input, value.contours, value.hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		//delete smaller than 40 contours
		for (vector<vector<Point> >::iterator filt = value.contours.begin(); filt != value.contours.end();) {
			if (filt->size()<40)
				filt = value.contours.erase(filt);
			else
				++filt;
		}
		ratio = (float*)malloc(value.contours.size());

		for (int i = 0; i < value.contours.size(); i++)
		{
			bool skip = false;
			fitLine(Mat(value.contours[i]), lines, CV_DIST_L2, 0, 1, 0.01);
			ratio[i] = lines[0] / lines[1];
			//cout << ratio[i]<< endl;
			for (int j = 0; j < i; j++) {
				if (abs(ratio[j] - ratio[i]) < 1) {
					skip = true;
					break;
				}
			}
			if (skip == false) {
				int y_down = 360;
				int y_up = 0;
				//lines for marking punkts from fitline
				line(output, Point(lines[2]-40, lines[3]), Point(lines[2]+40, lines[3]), Scalar(0, 255, 0), 2);
				line(output, Point(lines[2], lines[3]-40), Point(lines[2], lines[3]+40), Scalar(0, 255, 0), 2);

				int x_up = lines[0] / lines[1] * (y_up - lines[3]) + lines[2];
				int x_down = lines[0] / lines[1] * (y_down - lines[3]) + lines[2];
				line(output, Point(x_up, y_up), Point(x_down, y_down), Scalar(255, 0, 0), 2);
			}
			
		}
	}

	void showBirdView(Mat &input, Mat &output) {
		int rows = input.size[0];
		int cols = input.size[1];
		int ch = input.channels();
		
		//Point2f pts1[4] = { Point(0, 340), Point(100,100), Point(540,100), Point(640,0) };
		//Point2f pts2[4] = { Point(0, 340), Point(0,0), Point(640,0), Point(640,340) };
		//Point2f pts1[4] = { Point(0, 340), Point(540,100),Point(100,100),  Point(640,0) };
		//Point2f pts2[4] = { Point(0, 340), Point(640,0),Point(0,0), Point(640,340) };
		Point2f pts1[4] = { Point(100,70),Point(540,70),Point(0,360),Point(640,360) };
		Point2f pts2[4] = { Point(0,0),Point(640,0),Point(0,360),Point(640,360) };
		
		Mat M = getPerspectiveTransform(pts1, pts2);

		warpPerspective(input, output, M, Size(640, 360));
	}
	void edge(Mat &input, Mat &output, int thresh) {
		Mat kernel, thresh_out;
		kernel = Mat(1, 3, CV_32F);
		kernel.at<float>(0, 0) = -1;
		kernel.at<float>(0, 1) = 0;
		kernel.at<float>(0, 2) = 1;

		threshold(input, thresh_out, thresh, 255, THRESH_BINARY);

		Mat element = getStructuringElement(0, Size(15, 15), Point(7, 7));
		morphologyEx(thresh_out, thresh_out, 3, element);

		filter2D(thresh_out, output, -1, kernel, Point(-1, -1), 0, BORDER_DEFAULT);
	}
};

car_function car;

int main()
{
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat rgb(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat frame(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat img_gray(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat img_bird(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat img_canny(CAM_RES_Y, CAM_RES_X, CV_8UC1);

	Mat edge(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat img_contours(CAM_RES_Y, CAM_RES_X, CV_8UC3);
	Mat img_contours2(CAM_RES_Y, CAM_RES_X, CV_8UC3);
	Mat img_lanes(CAM_RES_Y, CAM_RES_X, CV_8UC3);

	
	const string file_name = "10.jpg";
	img = imread(file_name);
	if (!img.data) {
		cout << "Nie odnalezionu pliku " << file_name;
		return -1;
	}
	namedWindow("value", WINDOW_AUTOSIZE);
	int thresh = 200;

	createTrackbar("Thresh", "value", &thresh, 300, NULL);

	while (true) {

		car.convert_to_gray(img, img_gray);
		//car.showBirdView(img_gray,img_bird);
		car.make_blur(img_gray, img_gray);
		car.edge(img_gray, edge, thresh);

		//car.make_canny(img_gray, img_canny);

		//car.mask(img_canny, img_canny);
		//car.mask(edge, edge);
		car.mask(edge, edge);
		//car.fit_line(img_canny, img_contours);
		//namedWindow("fit line", CV_WINDOW_AUTOSIZE);
		//imshow("fit line", img_contours);

		//Mat img_contours2(CAM_RES_Y, CAM_RES_X, CV_8UC3);
		Mat img_contours2 = Mat::zeros(CAM_RES_Y, CAM_RES_X, CV_8UC3);
		car.contours(edge, img_contours2);
		namedWindow("approx poly", CV_WINDOW_AUTOSIZE);
		imshow("approx poly", img_contours2);

		namedWindow("input", CV_WINDOW_AUTOSIZE);
		imshow("input", img);
		waitKey(200);
	}
	waitKey(0);
	return(0);
}
