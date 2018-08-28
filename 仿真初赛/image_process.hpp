#pragma once
#ifndef  _IMAGE_PROCESS_H 
#define  _IMAGE_PROCESS_H 
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "AirLib/include/common/ImageCaptureBase.hpp"

#include <iostream>
#include  <direct.h>  
#include  <stdio.h> 
#include <Windows.h>
 
//图像处理模块
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>   
#include <opencv2/opencv.hpp>   
#include "preprocess.hpp"
 
using namespace cv;

using namespace msr::airlib;

typedef ImageCaptureBase::ImageResponse ImageResponse;
typedef ImageCaptureBase::ImageRequest ImageRequest;
void get_image(std::vector<ImageRequest>request, string path, std::vector<ImageRequest>request1, string path1, std::vector<ImageRequest>request2,
	int search_number, int sign, Pos &pos_expect, Pos &pos_true, Pos pos_init,float &x_control, float &y_control,float &high_control, Mat w1, Mat w2, Mat b1, Mat b2)
{
	msr::airlib::MultirotorRpcLibClient RpcLibClientBase;
	Mat Front_Scence_image(480, 640, CV_8UC3);
	Mat Front_Scence_image1(480, 640, CV_8UC3);
	Mat Below_Scence_image(480, 640, CV_8UC3);
	Mat Below_Scence_image1(480, 640, CV_8UC3);
	int recongtion_success  = 0;
	int recongtion_success1 = 0;
	float move_x = 0.0;
	float move_y = 0.0;
	int model =0;
	int sign_front = 0;
	Rect number_rect, number_rect1; int output = 0;
	int frame = 0;
	while (1)
	{
		const std::vector<ImageResponse>& response = RpcLibClientBase.simGetImages(request);
		for (const ImageResponse& image_info : response)
		{
			if (path != "")
			{
				//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
				std::string file_path = path;
				if (image_info.pixels_as_float)
				{
					Mat img3 = Mat(480, 640, CV_32FC1);
					memcpy(img3.data, image_info.image_data_float.data(), image_info.image_data_float.size() * sizeof(float));
					img3 = 1 - img3 / 255;
					//img3.copyTo(out);
				}
				else
				{
					std::ofstream file(path + std::to_string(1) + ".png", std::ios::binary);
					file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
					file.close();
					Front_Scence_image = imread(file_path + std::to_string(1) + ".png");
				}
			}
		}

		const std::vector<ImageResponse>& response1 = RpcLibClientBase.simGetImages(request1);
		for (const ImageResponse& image_info1 : response1)
		{
			if (path != "")
			{
				//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
				std::string file_path = path1;
				if (image_info1.pixels_as_float)
				{
 
				}
				else
				{
					std::ofstream file1(path1 + std::to_string(1) + ".png", std::ios::binary);
					file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
					file1.close();
					Below_Scence_image = imread(file_path + std::to_string(1) + ".png");
 
				}
			}
		}

		//const std::vector<ImageResponse>& response2 = RpcLibClientBase.simGetImages(request2);
		//for (const ImageResponse& image_info2 : response2)
		//{
		//	if (path != "")
		//	{
		//		//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
		//		std::string file_path = path1;
		//		if (image_info2.pixels_as_float)
		//		{
		//			Mat img3 = Mat(480, 640, CV_32FC1);
		//			memcpy(img3.data, image_info2.image_data_float.data(), image_info2.image_data_float.size() * sizeof(float));
		//			img3 = img3 / 255;
		//			cv::imshow("img3", img3);
		//			cv::waitKey(1);
		//		}
		//		else
		//		{
		//		}
		//	}
		//}

		Front_Scence_image.copyTo(Front_Scence_image1);
		Below_Scence_image.copyTo(Below_Scence_image1);

	 	if ((search_number == 1)|| (search_number == 2) || (search_number == 3))
		{
			model = 2;
		}
		else
		{
			//Number_recognition(0, Front_Scence_image, Front_Scence_image1, search_number, number_rect, w1, w2, b1, b2, recongtion_success);
			//Number_recognition(1, Below_Scence_image, Below_Scence_image1, search_number, number_rect1, w1, w2, b1, b2, recongtion_success1);
			//if (recongtion_success == 1)
			//{
			//	model = 1;
			//}
			//if (recongtion_success == 1)
			//{
			//	model = 2;
			//}
			//if ((recongtion_success != 1)&& (recongtion_success != 0))
			//{
			//	model = 0;
			//}

			if ((model != 1)&&(model != 2))
			{
			   model = 3;
			}
		}
		if (search_number == 11)
		{
			pos_expect.height = -1;
		}
		//前置模式
		if (model == 1)
		{
			Number_recognition(0, Front_Scence_image, Front_Scence_image1, search_number, number_rect, w1, w2, b1, b2, recongtion_success,frame);
			if (search_number == 4)
			{
				if ((recongtion_success == 1) && (sign == 0) && ((pos_expect.height - pos_true.height)<1)
					&& ((pos_expect.x - pos_true.x)<0.2) && ((pos_expect.y - pos_true.y)<0.2))
				{
					int offset_y = floor(number_rect.x + number_rect.width / 2) - 320;
					int offset_height = floor(number_rect.y + number_rect.height / 2) - 240;

					if ((30 <= abs(offset_y)) && (y_control < 0.001))
					{

						if (offset_y > 0)
						{
							pos_expect.y = pos_expect.y + 0.03;
						}
						else
						{
							pos_expect.y = pos_expect.y - 0.03;
						}
					}

					if ((30 <= abs(offset_height)))
					{
						if (offset_height > 0)
						{
							pos_expect.height = pos_expect.height - 0.03;
						}
						else
						{
							pos_expect.height = pos_expect.height + 0.03;
						}
					}
					if (number_rect.width*number_rect.height<6000)
					{
						pos_expect.x = pos_expect.x - 0.03;
					}
					if (number_rect.width*number_rect.height>10000)
					{
						pos_expect.x = pos_expect.x + 0.03;
					}
					if (((number_rect.width*number_rect.height > 6000) && (number_rect.width*number_rect.height < 10000) && (abs(offset_y) < 15)
						&& (abs(offset_height) < 15) && (x_control < 0.0008) && (y_control < 0.0008)))
					{
						sign = 1;
						pos_expect.height = 1.9 + pos_expect.height;
					}
				}
				if ((sign == 1) && ((pos_expect.height - pos_true.height) < 0.8) && (x_control < 0.0008) && (y_control < 0.0008))
				{
					pos_expect.x = pos_expect.x - 5;
					sign = 2;
				}
				if ((recongtion_success == 1) && (sign == 2) && (abs(pos_expect.x - pos_true.x)<0.2) && (abs(pos_expect.y - pos_true.y)<0.2) && (abs(pos_expect.height - pos_true.height)<0.8))
				{
					recongtion_success = 0;
					sign = 0;
					search_number++;
					pos_expect.height = pos_init.height;
					pos_expect.x = pos_true.x;
					pos_expect.y = pos_true.y;
					model = 0;
					sign_front = 0;
				}
			}
			else
			{
				if ((recongtion_success == 1) && (sign == 0) && ((pos_expect.height - pos_true.height)<1)
					&& ((pos_expect.x - pos_true.x)<0.2) && ((pos_expect.y - pos_true.y)<0.2))
				{
					int offset_y = floor(number_rect.x + number_rect.width / 2) - 320;
					int offset_height = floor(number_rect.y + number_rect.height / 2) - 240;

					if ((30 <= abs(offset_y)) && (y_control < 0.001))
					{

						if (offset_y > 0)
						{
							pos_expect.y = pos_expect.y + 0.03;
						}
						else
						{
							pos_expect.y = pos_expect.y - 0.03;
						}
					}

					if ((30 <= abs(offset_height)))
					{
						if (offset_height > 0)
						{
							pos_expect.height = pos_expect.height - 0.03;
						}
						else
						{
							pos_expect.height = pos_expect.height + 0.03;
						}
					}
					if (number_rect.width*number_rect.height<2000)
					{
						pos_expect.x = pos_expect.x - 0.03;
					}
					if (number_rect.width*number_rect.height>6000)
					{
						pos_expect.x = pos_expect.x + 0.03;
					}

					if (((number_rect.width*number_rect.height > 2000) && (number_rect.width*number_rect.height < 6000) && (abs(offset_y) < 15)
						&& (abs(offset_height) < 15) && (x_control < 0.0008) && (y_control < 0.0008)))
					{
						sign = 1;
						pos_expect.height = 1.9 + pos_expect.height;
					}
				}
				if ((sign == 1) && ((pos_expect.height - pos_true.height) < 0.8) && (x_control < 0.0008) && (y_control < 0.0008))
				{
					pos_expect.x = pos_expect.x -5;
					sign = 2;
				}
				if ((recongtion_success == 1) && (sign == 2) && (abs(pos_expect.x - pos_true.x)<0.2) && (abs(pos_expect.y - pos_true.y)<0.2) && (abs(pos_expect.height - pos_true.height)<0.8))
				{
					recongtion_success = 0;
					sign = 0;
					search_number++;
					pos_expect.height = pos_init.height;
					pos_expect.x = pos_true.x;
					pos_expect.y = pos_true.y;
					model = 0;
					sign_front = 0;
				}
			}
		}
		//下置模式
		if (model == 2)
		{
			Number_recognition(1, Below_Scence_image, Below_Scence_image1, search_number, number_rect1, w1, w2, b1, b2, recongtion_success1,frame);
			if ((recongtion_success1 == 1) && (sign == 0) && ((pos_expect.height - pos_true.height)<0.2)&&(number_rect1.width*number_rect1.height>300))
			{
				int offset_y = floor(number_rect1.x + number_rect1.width / 2) - 320;
				move_y = (float)0.03*offset_y;
				pos_expect.y = pos_expect.y + move_y;
				sign = 1;
				int offset_x = floor(number_rect1.y + number_rect1.height / 2) - 240;
				move_x = (float)0.03*offset_x;
				pos_expect.x = pos_expect.x + move_x;
				pos_expect.height = pos_init.height + 8;
			}
			if ((recongtion_success1 == 1) && (sign == 1))
			{
				int offset_y = floor(number_rect1.x + number_rect1.width / 2) - 320;
				int offset_x = floor(number_rect1.y + number_rect1.height / 2) - 240;
				if ((abs(offset_y) <10) && (abs(offset_x) <10) && (x_control<0.0008) && (y_control<0.0008))
				{
					if (number_rect1.width*number_rect1.height < 2500)
					{
						pos_expect.height = pos_init.height +3;
					}
					else
					{			
						  pos_expect.height = -1;
					      sign = 2;
					}

				}
				if ((10 <= abs(offset_y)) && (y_control<0.0008) && (pos_expect.height != -1))
				{
					if (offset_y > 0)
					{
						pos_expect.y = pos_expect.y + 0.04;
					}
					else
					{
						pos_expect.y = pos_expect.y - 0.04;
					}
				}
				if ((10 <= abs(offset_x)) && (x_control<0.0008) && (pos_expect.height != -1))
				{
					if (offset_x > 0)
					{
						pos_expect.x = pos_expect.x + 0.04;
					}
					else
					{
						pos_expect.x = pos_expect.x - 0.04;
					}
				}
 
			}
			if ((recongtion_success1 == 1) && (sign == 2) && (pos_true.height < 9.5))
			{
				recongtion_success1 = 0;
				sign = 0;
				search_number++;
				pos_expect.height = pos_init.height + 20;
				pos_expect.x = pos_true.x;
				pos_expect.y = pos_true.y;
				model = 0;
				sign_front = 0;
			}
		}
		//搜寻模式
		if ((model == 3) && (search_number != 10))
		{
			if (sign_front == 0)
			{
				pos_expect.height = pos_init.height + 2.5;
				sign_front = 1;
			}
			if (sign_front == 1)
			{
				if ((abs(pos_expect.height - pos_true.height) < 1) && (x_control < 0.0008) && (y_control < 0.0008))
				{
					pos_expect.x = pos_expect.x - 6;
					pos_expect.y = pos_init.y + 15;
					sign_front = 2;
				}			
			}
			if (sign_front == 2)
			{
				Number_recognition(0, Front_Scence_image, Front_Scence_image1, search_number, number_rect, w1, w2, b1, b2, recongtion_success,frame);
				if ((recongtion_success == 1)&& (number_rect.width*number_rect.height > 100))
				{
					if ((number_rect.x + number_rect.width / 2 < 360) && (number_rect.x - number_rect.width / 2 > 280))
					{
							recongtion_success = 0;
							sign = 0;
							pos_expect.height = pos_true.height;
							pos_expect.x = pos_true.x;
							pos_expect.y = pos_true.y;
							model = 1;
					}	
					else
					{
						recongtion_success = 0;
					}
					cv::putText(Front_Scence_image1, "sign_front" + to_string(sign_front),
						Point(60, 100), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				else
				{
					if ((pos_true.y>0) && (abs(pos_expect.y - pos_true.y)< 0.5))
					{
						pos_expect.y = pos_init.y - 15;
					}
					if ((pos_true.y<0) && (abs(pos_expect.y - pos_true.y)< 0.5))
					{
						pos_expect.y = pos_init.y;
						pos_expect.height = pos_init.height + 15;
						pos_expect.x = pos_true.x;
						model = 2;

					}
					cv::putText(Front_Scence_image1, "sign_front" + to_string(sign_front),
						Point(100, 100), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字
				}
				recongtion_success == 0;
			}

		}
		if ((search_number == 10)&&(model==0))
		{
			Number_recognition(1, Below_Scence_image, Below_Scence_image1, search_number, number_rect1, w1, w2, b1, b2, recongtion_success1, frame);
			recongtion_success = 0;
			sign = 0;
			pos_expect.height = pos_init.height+20;
			pos_expect.x = pos_true.x;
			pos_expect.y = pos_init.y;
			model = 2;
		}





		cv::putText(Front_Scence_image1, "NOW Number："+to_string(search_number),
			Point(10,10), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字

		cv::putText(Front_Scence_image1, "Model" + to_string(model),
			Point(30, 30), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字

		cv::putText(Front_Scence_image1, "Success" + to_string(recongtion_success),
			Point(60, 60), CV_FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 0), 1, 4);//在图片上写文字

		cv::imshow("前置摄像头", Front_Scence_image1);
		cv::imshow("下置摄像头", Below_Scence_image1);
		cv::waitKey(1);
		frame++;
 
   }
}




#endif