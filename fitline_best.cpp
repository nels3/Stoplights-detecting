
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

#define CAM_RES_X 640
#define CAM_RES_Y 360

RNG rng(12345);

//mask
class mask {
public:
	int maskx1 = 0;
	int masky1 = CAM_RES_Y;
	int maskx2 = 0;
	int masky2 = 240;

	int maskx3 = 50;
	int masky3 = 50;
	int maskx4 = CAM_RES_X - maskx3;
	int masky4 = masky3;

	int maskx5 = CAM_RES_X;
	int masky5 = masky2;
	int maskx6 = CAM_RES_X;
	int masky6 = CAM_RES_Y;
	
	Point mask_trapez[6] ={
		Point(maskx1, masky1),
		Point(maskx2, masky2),
		Point(maskx3, masky3),
		Point(maskx4, masky4),
		Point(maskx5, masky5),
		Point(maskx6, masky6)
	};
	Point mask_trapez_1[4] = {
		Point(0, 320),
		Point(0,250),
		Point(600,250),
		Point(600,320)
	};
	Point mask_trapez_2[4] = {
		Point(0,250),
		Point(100,200),
		Point(500,200),
		Point(600,250)
	};
	Point mask_trapez_3[4] = {
		Point(100,200),
		Point(150,150),
		Point(450,150),
		Point(500,200)
	};
	Point mask_trapez_4[4] = {
		Point(150,150),
		Point(250,100),
		Point(640-250,100),
		Point(450,150)
	};
	Point mask_trapez_5[4] = {
		Point(150,100),
		Point(250,50),
		Point(640 - 250,50),
		Point(450,100)
	};
	Point mask_trapez_6[4] = {
		Point(150,100),
		Point(250,50),
		Point(640 - 250,50),
		Point(450,100)
	};
	void draw_y_mask(Mat &input) {
		line(input, Point(0, mask_trapez_1[0].y), Point(CAM_RES_X, mask_trapez_1[0].y), Scalar(0, 255, 0),2);
		line(input, Point(0, mask_trapez_2[0].y), Point(CAM_RES_X, mask_trapez_2[0].y), Scalar(0, 220, 0),2);
		line(input, Point(0, mask_trapez_3[0].y), Point(CAM_RES_X, mask_trapez_3[0].y), Scalar(0, 200,0), 2);
		line(input, Point(0, mask_trapez_4[0].y), Point(CAM_RES_X, mask_trapez_4[0].y), Scalar(0, 180, 0), 2);
		line(input, Point(0, mask_trapez_5[0].y), Point(CAM_RES_X, mask_trapez_5[0].y), Scalar(0, 100, 0), 2);
		line(input, Point(0, mask_trapez_6[0].y), Point(CAM_RES_X, mask_trapez_6[0].y), Scalar(0, 80, 0), 2);
	}
};

mask filt_mask;

class car_data {
public:
	double offset;
	int thresh = 100;
	vector<vector<Point> > contours;

	vector<vector<Point> > contours_approx;
	vector<Point> approx;
	vector<Vec4i> hierarchy;
	Vec4f fit;
	
	struct line_values {
		Point wspol = Point(640,340);
		float ratio;
		bool found;
	};
	line_values left;
	line_values right;
	int choice;

};
car_data cdata;

vector<Point> contoursConvexHull(vector<vector<Point> > contours)
{
	vector<Point> result;
	vector<Point> pts;
	for (size_t i = 0; i < contours.size(); i++)
		for (size_t j = 0; j < contours[i].size(); j++)
			pts.push_back(contours[i][j]);
	convexHull(pts, result);
	return result;
}

class car_function {
public:
	void canny_fun(Mat &input, Mat &output) {
		Canny(input, output, cdata.thresh, cdata.thresh * 2, 3);
	}
	void find_contours(Mat &input) {
		findContours(input, cdata.contours, cdata.hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
		for (vector<vector<Point> >::iterator filt = cdata.contours.begin(); filt != cdata.contours.end();) {
			if (filt->size()<20)
				filt = cdata.contours.erase(filt);
			else {
				++filt;
			}
		}
	}
	void draw_contours(Mat &input) {
		for (int i = 0; i < cdata.contours.size(); i++)
		{
			drawContours(input, cdata.contours, i, Scalar(255,0,0), 2, 8, cdata.hierarchy, 0, Point());
		}
	}
	void draw_contours_approx(Mat &input) {
		for (int i = 0; i < cdata.approx.size() - 1; i++)
		{
			line(input, cdata.approx.at(i), cdata.approx.at(i + 1), cvScalar(0, 0, 255), cdata.approx.size());
		}
	}
	void mask(Mat &input, Mat &output) {
		Mat mask = Mat::zeros(cv::Size(640, 360), CV_8UC1);
		fillConvexPoly(mask, filt_mask.mask_trapez, 6, cv::Scalar(255, 0, 0));
		//imshow("img", mask);
		bitwise_and(input, mask, output);
	}

	void mask_sparter(Mat &input, Mat &output, Point mask_point[4]) {
		Mat mask = Mat::zeros(cv::Size(640, 360), CV_8UC1);
		fillConvexPoly(mask, mask_point, 4, cv::Scalar(255, 0, 0));
		//imshow("img", mask);
		bitwise_and(input, mask, output);
	}
	
	void contours_approx(Mat &input, vector<vector<Point>> countours, Mat &output) {
		findContours(input, countours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		//usuwanie mniejszych konkturow niz 20		
		for (vector<vector<Point> >::iterator filt = countours.begin(); filt != countours.end();) {
			if (filt->size()<20)
				filt = countours.erase(filt);
			else {
				++filt;
			}
		}
		Vec4f lines;

		float * ratio;
		ratio = (float*)malloc(countours.size());
		
		for (int i = 0; i < countours.size(); i++)
		{
			approxPolyDP(Mat(countours[i]), cdata.approx, 1, true);
			draw_contours_approx(output);
			fitLine(Mat(countours[i]), lines, CV_DIST_L12, 0, 0.01, 0.01);
			ratio[i] = lines[0] / lines[1];
			bool skip = false;
			for (int j = 0; j < i; j++) {
				if (abs(ratio[j] - ratio[i]) < 0.2) {
					skip = true;
					break;
				}
			}
			if (skip == false) {
				//oznaczenie punktow zwrotnych
				line(output, Point(lines[2] - 100, lines[3]), Point(lines[2] + 100, lines[3]), Scalar(0, 255, 0), 2);
				line(output, Point(lines[2], lines[3] - 100), Point(lines[2], lines[3] + 100), Scalar(0, 255, 0), 2);
			
				int lefty = (-lines[2] * lines[1] / lines[0]) + lines[3];
				int righty = ((output.cols - lines[2])*lines[1] / lines[0]) + lines[3];

				line(output, Point(output.cols - 1, righty), Point(0, lefty), Scalar(255, 0, 0), 2);
			}
		}
	}
	void first_line(int x, int y, float ratio) {
		if (x > CAM_RES_X / 2) {
			cdata.right.wspol.x = x;
			cdata.right.wspol.y = y;
			cdata.right.ratio = ratio;
		}
		else if (x < CAM_RES_X / 2) {
			cdata.left.wspol.x = x;
			cdata.left.wspol.y = y;
			cdata.right.ratio = ratio;
		}

	}
	void know_lines(int x, int y, float ratio, Mat &input) {
		
		if (cdata.choice == 3) {
			if (abs(cdata.left.wspol.x - x) < 100) {
				line(input, Point(cdata.left.wspol), Point(x, y), Scalar(255, 255, 0), 2);
				cdata.left.wspol.x = x;
				cdata.left.wspol.y = y;
				cdata.left.ratio = ratio;
			}
			else if (abs(cdata.right.wspol.x - x) < 100) {
				line(input, Point(cdata.right.wspol), Point(x, y), Scalar(255, 0, 0), 2);
				cdata.right.wspol.x = x;
				cdata.right.wspol.y = y;
				cdata.right.ratio = ratio;
			}
			else if (x > CAM_RES_X / 2) {

				line(input, Point(cdata.right.wspol), Point(x, y), Scalar(255, 0, 0), 2);
				cdata.right.wspol.x = x;
				cdata.right.wspol.y = y;
				cdata.right.ratio = ratio;
			}
			else if (x < CAM_RES_X / 2) {

				line(input, Point(cdata.left.wspol), Point(x, y), Scalar(255, 255, 0), 2);
				cdata.left.wspol.x = x;
				cdata.left.wspol.y = y;
				cdata.left.ratio = ratio;
			}
		}
		else if (cdata.choice = 2) {  //prawy
			line(input, Point(cdata.right.wspol), Point(x, y), Scalar(255, 0, 0), 2);
			cdata.right.wspol.x = x;
			cdata.right.wspol.y = y;
			cdata.right.ratio = ratio;
		}
		else if (cdata.choice = 1) {
			line(input, Point(cdata.left.wspol), Point(x, y), Scalar(255, 255, 0), 2);
			cdata.left.wspol.x = x;
			cdata.left.wspol.y = y;
			cdata.left.ratio = ratio;
		}
		else
			cout << "cos nie tak" << endl;
	}

	void contours_approx_smarter(Mat &input, vector<vector<Point>> countours, Mat &output, Point mask_punkt[4]) {
		findContours(input, countours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		for (vector<vector<Point> >::iterator filt = countours.begin(); filt != countours.end();) {
			//cout << filt->size() << endl;
			if (filt->size()<100)
				filt = countours.erase(filt);
			else {
				cout << filt->size() << endl;
				++filt;
			}
			//cout << filt->size() << endl;
		}
		Vec4f lines;
		cdata.contours.resize(cdata.approx.size());
		float ratio[20] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
		bool skip = false;
		for (int i = 0; i < countours.size(); i++)
		{
			//approxPolyDP(Mat(countours[i]), cdata.approx, 1, true);
			//approxPolyDP(Mat(countours[i]), countours[i], 1, true);

			//draw_contours_approx(output);
			fitLine(Mat(countours[i]), lines, 1, 0, 0.01, 0.01);
			ratio[i] = lines[0] / lines[1];
			for (int j = 0; j < i; j++) {
				if (abs(ratio[j] - ratio[i]) < 1) {
					skip = true;
					break;
				}
			}
			if (skip == false) {
				int y_down = mask_punkt[0].y;
				int y_up = mask_punkt[1].y;

				int x_up = lines[0] / lines[1] * (y_up - lines[3]) + lines[2];
				int x_down = lines[0] / lines[1] * (y_down - lines[3]) + lines[2];
				
				if (x_up < CAM_RES_X / 2 && cdata.left.found == false) {
					line(output, Point(x_up, y_up), Point(x_down, y_down), Scalar(255, 255, 0), 2);
					first_line(x_up,y_up,ratio[i]);
					cdata.left.found = true;
					
				}
				else if (x_up >= CAM_RES_X / 2 && cdata.right.found == false) {
					line(output, Point(x_up, y_up), Point(x_down, y_down), Scalar(255, 0, 0), 2);
					first_line(x_up, y_up, ratio[i]);
					cdata.right.found = true;
				}
				if (cdata.choice != 3) {
					if (cdata.right.found == true and cdata.right.found == true)
						cdata.choice = 3;
					else if (cdata.right.found == true)
						cdata.choice = 2;
					else if (cdata.left.found == true)
						cdata.choice = 1;
					else
						cdata.choice = 0;
				}

				else// (x_up < CAM_RES_X / 2 && cdata.left.found == true)
					know_lines(x_up,y_up, ratio[i], output);
			//	if (x_up > CAM_RES_X / 2 && cdata.right.found == true)
					//know_lines(x_up, y_up, ratio[i], output);
				
			}
			
			skip = false;
		}
	}
	
	
};
car_function car;



int main( )
{
	Mat img(CAM_RES_Y, CAM_RES_X, CV_8UC4);
	Mat img_blur(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat canny_img(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat gray(CAM_RES_Y, CAM_RES_X, CV_8UC1);
	Mat drawing = Mat::zeros(canny_img.size(), CV_8UC3);

	const string file_name = "11.jpg";
	img = imread(file_name);
	if (!img.data) {
		cout << "Nie odnalezionu pliku " << file_name;
		return -1;
	}

	//while (true) {

		cvtColor(img, gray, CV_BGR2GRAY);
		blur(gray, img_blur, Size(3, 3));
		
		car.canny_fun(img_blur, canny_img);
		car.mask(canny_img, canny_img);
		car.find_contours(canny_img);
	
		car.contours_approx(canny_img, cdata.contours, drawing);
		
		namedWindow("without", CV_WINDOW_AUTOSIZE);
		imshow("without", drawing);
		namedWindow("img", CV_WINDOW_AUTOSIZE);
		imshow("img", img);
		//waitKey(200);
	//}
	waitKey(0);
	return(0);
}


