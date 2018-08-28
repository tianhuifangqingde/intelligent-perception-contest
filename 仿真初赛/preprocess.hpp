#pragma once
#ifndef  _PREPROCESS_H 
#define  _PREPROCESS_H 

#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>   
#include <opencv2/opencv.hpp>   

using namespace cv;
using namespace std;
//仿射变换
void Affine_transformation(Mat img, OutputArray out)
{

	Mat  img1, img2;
	//int frame = 385;
	cvtColor(img, img1, CV_RGB2GRAY);
	threshold(img1, img2, 180, 255, CV_THRESH_BINARY);
	img2.copyTo(out);
	float angle = 0;

	std::vector<std::vector<Point>>contours;
	std::vector<Vec4i>hierarchy;
	findContours(img2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

	int sign_max = 0;
	int number_max = 0;
	int Perimeter_max = 0; Point  Max_center;
	int Perimeter_number = 100000; Point  Number_center;
	std::vector<RotatedRect>minRect(contours.size());
	Point2f rect_points[4];
	if (contours.size() > 0)
	{
		for (int i = 0; i < contours.size(); i++)
		{
			minRect[i] = minAreaRect(Mat(contours[i]));
			minRect[i].points(rect_points);
			int width1 = sqrt((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x) +
				(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
			int height1 = sqrt((rect_points[1].x - rect_points[2].x)*(rect_points[1].x - rect_points[2].x) +
				(rect_points[1].y - rect_points[2].y)*(rect_points[1].y - rect_points[2].y));
			if (Perimeter_max < 2 * width1 + 2 * height1)
			{
				Perimeter_max = 2 * width1 + 2 * height1;
				sign_max = i;
			}
			if ((Perimeter_number > 2 * width1 + 2 * height1) && (2 * width1 + 2 * height1) > 200)
			{
				if ((width1 / height1 > 5) || (height1 / width1 > 5))
				{

				}
				else
				{
					Perimeter_number = 2 * width1 + 2 * height1;
					number_max = i;
				}
			}
		}

		minRect[sign_max].points(rect_points);//外围
		Max_center.x = 0; Max_center.y = 0;
		for (int j = 0; j < 4; j++)
		{
			line(img, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
			Max_center.x = Max_center.x + rect_points[j].x;
			Max_center.y = Max_center.y + rect_points[j].y;
		}
		Max_center.x = floor(Max_center.x / 4);
		Max_center.y = floor(Max_center.y / 4);

		minRect[number_max].points(rect_points);//数字
		Number_center.x = 0; Number_center.y = 0;
		for (int j = 0; j < 4; j++)
		{
			line(img, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
			Number_center.x = Number_center.x + rect_points[j].x;
			Number_center.y = Number_center.y + rect_points[j].y;
		}
		Number_center.x = floor(Number_center.x / 4);
		Number_center.y = floor(Number_center.y / 4);

		//circle(img, Max_center,10,Scalar(255,0,0));
		//	circle(img, Number_center, 10, Scalar(255, 0, 0));




		//angle =atan2(abs(Number_center.y- Max_center.y), abs(Number_center.x- Max_center.x))*57.3;

		angle = minRect[sign_max].angle;
		if ((Max_center.x < Number_center.x) && (Max_center.y < Number_center.y))
		{
			angle = angle;
		}
		if ((Max_center.x < Number_center.x) && (Max_center.y > Number_center.y))
		{
			angle = angle - 90;
		}
		if ((Max_center.x > Number_center.x) && (Max_center.y > Number_center.y))
		{
			angle = angle - 180;
		}
		if ((Max_center.x > Number_center.x) && (Max_center.y < Number_center.y))
		{
			angle = angle - 270;
		}
		//cout << angle << endl;

		Point2f center(floor(img2.cols / 2), floor(img2.rows / 2));
		Mat rot_mat = getRotationMatrix2D(center, angle, 1.0);
		cv::warpAffine(out, out, rot_mat, img2.size());
	}

}
//提取数字ROI
void Number_ROI1(Mat img, OutputArray img2, Rect &img_rect)
{
	//success = 1;
	Mat img4;
	img.copyTo(img4);
	std::vector<std::vector<Point> > contours;
	std::vector<Vec4i> hierarchy;
	findContours(img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	std::vector<Rect> boundRect(contours.size());
	int max = 0;
	int sign = -1;
	for (int i = 0; i < contours.size(); i++)
	{
		boundRect[i] = boundingRect(Mat(contours[i]));
		if ((boundRect[i].width*boundRect[i].height > 20 * 20))//大框
		{
			if ((float)boundRect[i].height / (float)boundRect[i].width > 1.2)
			{
				if (max < boundRect[i].width* boundRect[i].height)//小框
				{
					max = boundRect[i].width* boundRect[i].height;
					sign = i;
				}
			}
		}
	}

	if (sign != -1)
	{
		Mat img3;
		img3 = img4(Rect(boundRect[sign].x, boundRect[sign].y, boundRect[sign].width, boundRect[sign].height));
		img_rect = boundRect[sign];
		resize(img3, img2, Size(50, 50));
	}
	else
	{
		resize(img4, img2, Size(50, 50));
	}
}
void Number_ROI2(Mat img, OutputArray out, Rect &img_rect)
{
	Mat img2, img4;
	cvtColor(img, img2, CV_RGB2GRAY);
	//Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
	//erode(img3, img3, element);
	//imshow("erw", img3);

	uchar* pxvec = img2.ptr<uchar>(0);
	int i, j;
	for (i = 0; i < img2.rows; i++)
	{
		pxvec = img2.ptr<uchar>(i);
		for (j = 0; j < img2.cols; j++)
		{
			if ((pxvec[j*img2.channels()] > 140) && (pxvec[j*img.channels()] < 180))
			{
				pxvec[j] = 255;
			}
			else
			{
				pxvec[j] = 0;
			}

		}
	}
	img2.copyTo(img4);
	imshow("adfaddfadsdffad", img2);
	int sign = -1;
	std::vector<std::vector<Point>>contours;
	std::vector<Vec4i>hierarchy;
	findContours(img2, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
	std::vector<RotatedRect>minRect(contours.size());
	Point2f rect_points[4];
	if (contours.size() > 0)
	{
		for (int i = 0; i < contours.size(); i++)
		{
			minRect[i] = minAreaRect(Mat(contours[i]));
			minRect[i].points(rect_points);
			Rect a = minRect[i].boundingRect();
			if ((a.width > 20) && (a.height > 20) && (sign == -1))
			{
				if ((minRect[i].angle < 5) && (minRect[i].angle > -5))
				{
					float b = (float)a.height / (float)a.width;
					if ((b < 3) && (b > 1.3))
					{
						for (int j = 0; j < 4; j++)
						{
							line(img, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
							sign = i;
						}
					}

				}
			}
		}
	}



	if (sign != -1)
	{
		Mat img3;
		Rect boundRect = minRect[sign].boundingRect();
		img3 = img4(Rect(boundRect.x, boundRect.y, boundRect.width, boundRect.height));
		img_rect = boundRect;
		resize(img3, out, Size(50, 50));
	}
	else
	{
		resize(img4, out, Size(50, 50));
	}


}
//加载神经网络
void Load_neural_network(OutputArray w1, OutputArray w2, OutputArray b1, OutputArray b2)
{
	FileStorage fs("w1.xml", FileStorage::READ);
	Mat w11;
	fs["w1"] >> w11;
	transpose(w11, w1);
	fs.release();
	//w2
	FileStorage fs2("w2.xml", FileStorage::READ);
	Mat  w3;
	fs2["w2"] >> w3;
	w3 = w3.reshape(0, 25);
	w3.copyTo(w2);
	fs2.release();
	//b1
	Mat b11;
	FileStorage fs3("b1.xml", FileStorage::READ);
	fs3["b1"] >> b11;
	b11.copyTo(b1);
	fs3.release();
	//b2
	FileStorage fs4("b2.xml", FileStorage::READ);
	Mat b21;
	fs4["b2"] >> b21;
	b21.copyTo(b2);
	fs4.release();
}
//识别数字
int recognition(Mat img, Mat w1, Mat w2, Mat b1, Mat b2)
{
	Mat pic = img;
	Mat pic_two;

	Mat pic2;
	float image_feature1[25] = { 0 };
	threshold(pic, pic2, 200, 255, CV_THRESH_BINARY);

	int sum = 0;
	for (int j = 1; j < 6; j++)
	{
		for (int k = 1; k < 6; k++)
		{
			float sum = 0;
			for (int h = 0; h < 10; h++)
			{
				for (int l = 0; l < 10; l++)
				{
					sum += (pic2.at<uchar>((j - 1) * 10 + h, (k - 1) * 10 + l) / 255);
				}
			}
			image_feature1[(j - 1) * 5 + k - 1] = (100.0 - sum) / 100.0;
		}
	}
	Mat image_feature = Mat(25, 1, CV_32FC1, image_feature1);
	// cout << image_feature << endl;


	Mat hidden, hiddenout, val, out;
	float  hiddenout_s[25] = { 0 };
	hidden = w1*image_feature + b1;
	//cout << hidden << endl;
	for (int m = 0; m < 25; m++)
	{

		float a = -hidden.at<float>(m, 0);
		a = 1 / (1 + exp(a));
		hiddenout_s[m] = a;
	}
	hiddenout = Mat(1, 25, CV_32FC1, hiddenout_s);
	transpose(hiddenout*w2, val);
	out = val + b2;
	//cout << out << endl;

	float max = -10.0;
	float number = 11;
	for (int i = 0; i < 11; i++)
	{
		if (out.at<float>(i) > max)
		{
			max = out.at<float>(i);
			number = i + 1;
		}
	}

	return number;
}

void Number_recognition(int camera_id, Mat img, OutputArray out, int number_expect,Rect &img_rect, Mat w1, Mat w2, Mat b1, Mat b2,int &success,int frame)
{
	Mat img_two_value, img_rectangle, img_two_value_origin;
	img.copyTo(img_rectangle);
	if (camera_id == 0)
	{
		//imwrite("F:/AirSim-master6.26/image4/" + to_string(frame) + ".png",img);
		// 二值化           //原图画框    //二值化后原图
		cvtColor(img, img_two_value, CV_RGB2GRAY);
		
		threshold(img_two_value, img_two_value, 160, 255, CV_THRESH_BINARY);
		img_two_value.copyTo(img_two_value_origin);

		std::vector<std::vector<Point>>contours;
		std::vector<Vec4i>hierarchy;
		findContours(img_two_value, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		std::vector<RotatedRect>minRect(contours.size());
		Point2f rect_points[4];

		int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
		int num = 0;
		if (contours.size() > 0)
		{
			for (int i = 0; i < contours.size(); i++)
			{
				minRect[i] = minAreaRect(Mat(contours[i]));
				minRect[i].points(rect_points);

				int width1 = sqrt((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x) +
					(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
				int height1 = sqrt((rect_points[1].x - rect_points[2].x)*(rect_points[1].x - rect_points[2].x) +
					(rect_points[1].y - rect_points[2].y)*(rect_points[1].y - rect_points[2].y));

				if ((width1 > 8) && (height1 > 8) && (width1 < 600) && (height1 < 450) && (minRect[i].angle<5) && (minRect[i].angle>-5))
				{

					float b = (float)height1 / (float)width1;
					float a = (float)width1 / (float)height1;
					if (number_expect == 4)
					{
						if ((b <2) && (b > 1.3))
						{
							for (int j = 0; j < 4; j++)
							{
								line(img_rectangle, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
							}
							sign[num] = i;
							if (num<9) { num++; }
						}
					}
					else
					{
						if ((a <2) && (a > 1.3))
						{
							for (int j = 0; j < 4; j++)
							{
								line(img_rectangle, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
							}
							sign[num] = i;
							if (num<9) { num++; }
						}

					}



				}
			}
		}

		for (int i = 0; i < 10; i++)
		{
			if (sign[i] != -1)
			{
				Rect boundRect = minRect[sign[i]].boundingRect();
				if (boundRect.x <= 0) { boundRect.x = 1; }	if (boundRect.x >= 639) { boundRect.x = 639; }
				if (boundRect.y <= 0) { boundRect.y = 1; }	if (boundRect.y >= 479) { boundRect.y = 479; }
				if (boundRect.x + boundRect.width > 639) { boundRect.width = 639 - boundRect.x; }
				if (boundRect.y + boundRect.height > 479) { boundRect.height = 479 - boundRect.y; }

				Mat img_number;
				resize(img_two_value_origin(boundRect), img_number, Size(50, 50));
				int number = recognition(img_number, w1, w2, b1, b2);
				if (number != 11)
				{
					putText(img_rectangle, to_string(number) + " " + to_string(boundRect.x) + " " + to_string(boundRect.y) + " "
						+ to_string(boundRect.width) + " " + to_string(boundRect.height),
						Point(boundRect.x, boundRect.y), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				if (number_expect == number)
				{
					//imwrite("F:/AirSim-master6.26/image4/" + to_string(frame) + ".png", img_number);
					img_rect = boundRect;
					success = 1;
				}
			}
		}
	}

	if (camera_id == 1)
	{
		// 二值化           //原图画框    //二值化后原图
		cvtColor(img, img_two_value, CV_RGB2GRAY);
		threshold(img_two_value, img_two_value, 150, 255, CV_THRESH_BINARY);
		img_two_value.copyTo(img_two_value_origin);

		std::vector<std::vector<Point>>contours;
		std::vector<Vec4i>hierarchy;
		findContours(img_two_value, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		std::vector<RotatedRect>minRect(contours.size());
		Point2f rect_points[4];

		int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
		int num = 0;
		if (camera_id == 1)
		{
			if (contours.size() > 0)
			{
				for (int i = 0; i < contours.size(); i++)
				{
					minRect[i] = minAreaRect(Mat(contours[i]));
					minRect[i].points(rect_points);

					int width1 = sqrt((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x) +
						(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
					int height1 = sqrt((rect_points[1].x - rect_points[2].x)*(rect_points[1].x - rect_points[2].x) +
						(rect_points[1].y - rect_points[2].y)*(rect_points[1].y - rect_points[2].y));

					if ((width1 > 8) && (height1 > 8) && (width1 < 600) && (height1 < 450) && (minRect[i].angle<5) && (minRect[i].angle>-5))
					{

						float b = (float)height1 / (float)width1;
						float a = (float)width1 / (float)height1;
						if ((a <2) && (a > 1.3))
						{
							for (int j = 0; j < 4; j++)
							{
								line(img_rectangle, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
							}
							sign[num] = i;
							if (num<9) { num++; }
						}

					}
				}
			}
		}
		for (int i = 0; i < 10; i++)
		{
			if (sign[i] != -1)
			{
				Rect boundRect = minRect[sign[i]].boundingRect();
				if (boundRect.x <= 0) { boundRect.x = 1; }	if (boundRect.x >= 639) { boundRect.x = 639; }
				if (boundRect.y <= 0) { boundRect.y = 1; }	if (boundRect.y >= 479) { boundRect.y = 479; }
				if (boundRect.x + boundRect.width > 639) { boundRect.width = 639 - boundRect.x; }
				if (boundRect.y + boundRect.height > 479) { boundRect.height = 479 - boundRect.y; }

				Mat img_number;
				resize(img_two_value_origin(boundRect), img_number, Size(50, 50));
				int number = recognition(img_number, w1, w2, b1, b2);
				if (number != 11)
				{
					putText(img_rectangle, to_string(number) + " " + to_string(boundRect.x) + " " + to_string(boundRect.y) + " "
						+ to_string(boundRect.width) + " " + to_string(boundRect.height),
						Point(boundRect.x, boundRect.y), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				if (number_expect == number)
				{
					img_rect = boundRect;
					success = 1;
				}
			}
		}
	}


	img_rectangle.copyTo(out);
	 
}


void Number_recognition4(int camera_id, Mat img, OutputArray out, int number_expect, Rect &img_rect, Mat w1, Mat w2, Mat b1, Mat b2, int &success)
{
	Mat img_two_value, img_rectangle, img_two_value_origin;
	img.copyTo(img_rectangle);
	if (camera_id == 0)
	{
		// 二值化           //原图画框    //二值化后原图
		cvtColor(img, img_two_value, CV_RGB2GRAY);
		threshold(img_two_value, img_two_value, 100, 255, CV_THRESH_BINARY);
		img_two_value.copyTo(img_two_value_origin);
		imshow("二值化",img_two_value_origin);
		std::vector<std::vector<Point>>contours;
		std::vector<Vec4i>hierarchy;
		findContours(img_two_value, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		std::vector<RotatedRect>minRect(contours.size());
		Point2f rect_points[4];

		int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
		int num = 0;
		if (contours.size() > 0)
		{
			for (int i = 0; i < contours.size(); i++)
			{
				minRect[i] = minAreaRect(Mat(contours[i]));
				minRect[i].points(rect_points);

				int width1 = sqrt((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x) +
					(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
				int height1 = sqrt((rect_points[1].x - rect_points[2].x)*(rect_points[1].x - rect_points[2].x) +
					(rect_points[1].y - rect_points[2].y)*(rect_points[1].y - rect_points[2].y));

				if ((width1 > 8) && (height1 > 8) && (width1 < 600) && (height1 < 450) && (minRect[i].angle<5) && (minRect[i].angle>-5))
				{

					float b = (float)height1 / (float)width1;
					float a = (float)width1 / (float)height1;
					if ((a <2) && (a > 1.3))
					{
						for (int j = 0; j < 4; j++)
						{
							line(img_rectangle, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
						}
						sign[num] = i;
						if (num<9) { num++; }
					}
				}
			}
		}

		for (int i = 0; i < 10; i++)
		{
			if (sign[i] != -1)
			{
				Rect boundRect = minRect[sign[i]].boundingRect();
				if (boundRect.x <= 0) { boundRect.x = 1; }	if (boundRect.x >= 639) { boundRect.x = 639; }
				if (boundRect.y <= 0) { boundRect.y = 1; }	if (boundRect.y >= 479) { boundRect.y = 479; }
				if (boundRect.x + boundRect.width > 639) { boundRect.width = 639 - boundRect.x; }
				if (boundRect.y + boundRect.height > 479) { boundRect.height = 479 - boundRect.y; }

				Mat img_number;
				resize(img_two_value_origin(boundRect), img_number, Size(50, 50));
				int number = recognition(img_number, w1, w2, b1, b2);
				if (number != 11)
				{
					putText(img_rectangle, to_string(number) + " " + to_string(boundRect.x) + " " + to_string(boundRect.y) + " "
						+ to_string(boundRect.width) + " " + to_string(boundRect.height),
						Point(boundRect.x, boundRect.y), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				if (number_expect == number)
				{
					img_rect = boundRect;
					success = 1;
				}
			}
		}
	}

	if (camera_id == 1)
	{
		// 二值化           //原图画框    //二值化后原图
		cvtColor(img, img_two_value, CV_RGB2GRAY);
		threshold(img_two_value, img_two_value, 150, 255, CV_THRESH_BINARY);
		img_two_value.copyTo(img_two_value_origin);

		std::vector<std::vector<Point>>contours;
		std::vector<Vec4i>hierarchy;
		findContours(img_two_value, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

		std::vector<RotatedRect>minRect(contours.size());
		Point2f rect_points[4];

		int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
		int num = 0;
		if (camera_id == 1)
		{
			if (contours.size() > 0)
			{
				for (int i = 0; i < contours.size(); i++)
				{
					minRect[i] = minAreaRect(Mat(contours[i]));
					minRect[i].points(rect_points);

					int width1 = sqrt((rect_points[0].x - rect_points[1].x)*(rect_points[0].x - rect_points[1].x) +
						(rect_points[0].y - rect_points[1].y)*(rect_points[0].y - rect_points[1].y));
					int height1 = sqrt((rect_points[1].x - rect_points[2].x)*(rect_points[1].x - rect_points[2].x) +
						(rect_points[1].y - rect_points[2].y)*(rect_points[1].y - rect_points[2].y));

					if ((width1 > 8) && (height1 > 8) && (width1 < 600) && (height1 < 450) && (minRect[i].angle<5) && (minRect[i].angle>-5))
					{

						float b = (float)height1 / (float)width1;
						float a = (float)width1 / (float)height1;
						if ((a <2) && (a > 1.3))
						{
							for (int j = 0; j < 4; j++)
							{
								line(img_rectangle, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 1, 8);
							}
							sign[num] = i;
							if (num<9) { num++; }
						}

					}
				}
			}
		}
		for (int i = 0; i < 10; i++)
		{
			if (sign[i] != -1)
			{
				Rect boundRect = minRect[sign[i]].boundingRect();
				if (boundRect.x <= 0) { boundRect.x = 1; }	if (boundRect.x >= 639) { boundRect.x = 639; }
				if (boundRect.y <= 0) { boundRect.y = 1; }	if (boundRect.y >= 479) { boundRect.y = 479; }
				if (boundRect.x + boundRect.width > 639) { boundRect.width = 639 - boundRect.x; }
				if (boundRect.y + boundRect.height > 479) { boundRect.height = 479 - boundRect.y; }

				Mat img_number;
				resize(img_two_value_origin(boundRect), img_number, Size(50, 50));
				int number = recognition(img_number, w1, w2, b1, b2);
				if (number != 11)
				{
					putText(img_rectangle, to_string(number) + " " + to_string(boundRect.x) + " " + to_string(boundRect.y) + " "
						+ to_string(boundRect.width) + " " + to_string(boundRect.height),
						Point(boundRect.x, boundRect.y), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				if (number_expect == number)
				{
					img_rect = boundRect;
					success = 1;
				}
			}
		}
	}


	img_rectangle.copyTo(out);

}


 

#endif