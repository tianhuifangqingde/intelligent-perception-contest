 // Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/StandAloneSensors.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "AirLib/include/api/RpcLibClientBase.hpp"
#include <iostream>
#include <chrono>
#include <conio.h>
#include <thread>
//���˻�����ģ��
#include "PID.h"
#include "contrl.h"
#include "GPS_ECEF.h"
#include "IMU1.hpp" 
#include "Filter.hpp"
//ͼ����ģ��
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/core/core.hpp>   
#include <opencv2/opencv.hpp>   
#include "preprocess.hpp"
#include "image_process.hpp"
using namespace cv;
 
 #define RADIAN 57.29577913f




int main()
{
	using namespace msr::airlib;
	msr::airlib::MultirotorRpcLibClient RpcLibClientBase;
 	msr::airlib::RpcLibClientBase RpcLibClientBase_Image;
	typedef ImageCaptureBase::ImageRequest ImageRequest;
	typedef ImageCaptureBase::ImageResponse ImageResponse;
	typedef ImageCaptureBase::ImageType ImageType;
	typedef common_utils::FileSystem FileSystem;

	try {

		RpcLibClientBase.getConnectionState();
		RpcLibClientBase.confirmConnection();
		std::cout << "����������" << std::endl;
		RpcLibClientBase.isApiControlEnabled();
		RpcLibClientBase.enableApiControl(true);
		std::cout << "ͼ��APIʹ��" << std::endl;

		//���˻�����ģ��
		//�������˻�
		RpcLibClientBase.armDisarm(true);
		//�����ʹ��

		std::cout << "���FPVͼ��" << std::endl;
		std::vector<ImageRequest> request0 = { ImageRequest(0, ImageType::Scene)};//ǰ��ͼ
		std::string path = "F:\\AirSim-master6.26\\image\\";
		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
		std::vector<ImageRequest> request1 = { ImageRequest(3, ImageType::Scene) };//����ͼ
		std::string path1 = "F:\\AirSim-master6.26\\image\\";
		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
	 	std::vector<ImageRequest> request2 = { ImageRequest(0, ImageType::DepthPerspective,true) };//ǰ�����ͼ
		std::string path2 = "F:\\AirSim-master6.26\\image\\";
		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
		//���
		{
			//���
			float takeoffTimeout = 1;   //��ɵȴ�ʱ��
			RpcLibClientBase.takeoff(takeoffTimeout);
			RpcLibClientBase.hover();   		//�л�����ͣģʽ	
		}


		int frame = 0;//�����֡��

		//���GPSλ��
		auto Gpsposition = RpcLibClientBase.getGpsLocation();
		Gpsposition.altitude = abs(Gpsposition.altitude);
		std::cout << "����GPS " << std::endl;
		//��ȡ��ѹ������  ������θ߶�  ��ѹ��
		BarometerData planeBarometerData = RpcLibClientBase.getBarometerdata(); //����
		std::cout << "������ѹ�� " << std::endl;
		//��ô���������   x,y,z
		MagnetometerData planeMagnetometerData = RpcLibClientBase.getMagnetometerdata();
		std::cout << "��ȡ��������Ϣ" << std::endl;
		//��ùߵ�����     
		ImuData planeImuData = RpcLibClientBase.getImudata();
		std::cout << "��ȡ�ߵ�����" << std::endl;

		//��ʼλ��
		auto Gpsposition_init = Gpsposition;
		Radius radius;
		radius = CalcEarthRadius(Gpsposition.latitude);
		Pos pos, pos_init;
		pos_init = Offset(Gpsposition.latitude, Gpsposition.longitude, Gpsposition_init.latitude, Gpsposition_init.longitude, radius.r1, radius.r2);
		pos = pos_init;

		std::cout << "X��ʼ���꣺" << pos_init.x << "Y��ʼ���꣺" << pos_init.y << std::endl;
		float groud = planeBarometerData.altitude;
	    pos_init.height = groud;
		float high_control = (float)0.56;
		//float x_control = 0;
		//float y_control = 0;
		int sign2 = 1;
		Attitude drone;
		drone.pitch = (float)0.0; drone.roll = (float)0.0; drone.yaw = (float)0.0;
		float pitch_expect = (float)0.0; float roll_expect = (float)0.0; float yaw_expect = (float)0.0;
		float pitch_control = (float)0.0; 	float roll_control = (float)0.0; float yaw_control = (float)0.0;
		Attitude Drone;
		float x_control =(float)0.0;
		float y_control =(float)0;

		Pos move,Posistion_expect;
		move.x = (float)0.0;
		move.y = (float)0.0;
		move.height = (float)15.0;
		Posistion_expect = move;

		Pos    pos_filter = pos_init;
	    pos_filter.height = planeBarometerData.altitude;
		//ͼ��

		cv::Mat w1, w2, b1, b2;
		Load_neural_network(w1, w2, b1, b2);
	 
		int sign = 0;

		Posistion_expect.x = pos_init.x + move.x;
		Posistion_expect.y = pos_init.y + move.y;
		Posistion_expect.height = pos_init.height + move.height;

		int number1=1;

         
		std::thread t(get_image, request0, path, request1 ,path1,request2,number1,sign,std::ref(Posistion_expect),std::ref(pos_filter),
		    std::ref(pos_init), std::ref(x_control), std::ref(y_control), std::ref(high_control),  w1,  w2,  b1,  b2); // Acceptable.


		Posistion_expect.x = 0;//-47,-7        6����
		Posistion_expect.y = 0;//-24,14        4����
		while (1)
		{

			//double timeStart = (double)getTickCount();
			//double nTime = ((double)getTickCount() - timeStart) / getTickFrequency();
			//cout << "ʱ�䣺" << nTime << endl;

			Gpsposition = RpcLibClientBase.getGpsLocation();	
			planeBarometerData.altitude=  RpcLibClientBase.getBarometerdata().altitude;//�߶�δ�˲�
			pos_filter.height= (float)planeBarometerData_filter(pos_filter.height,planeBarometerData.altitude,0.2);
			planeMagnetometerData = RpcLibClientBase.getMagnetometerdata();		
			planeImuData = RpcLibClientBase.getImudata();//��ʱ3ms
 
		

			pos = Offset(Gpsposition.latitude, Gpsposition.longitude, Gpsposition_init.latitude, Gpsposition_init.longitude, radius.r1, radius.r2);
			pos_filter.x = (double)position_filter_x(pos_filter.x,pos.x,0.1);
		    pos_filter.y = (double)position_filter_x(pos_filter.y, pos.y,0.1);

			////ˮƽ����
			x_control = keep_horizontal_x((float)Posistion_expect.x, (float)pos_filter.x, (float)0.003, (float)0.001, (float)0.3);
			y_control = keep_horizontal_y((float)Posistion_expect.y, (float)pos_filter.y, (float)0.003, (float)0.001, (float)0.3);
			//IMU  
			Drone =IMUupdate(0.02f, planeImuData.angular_velocity.x()*RADIAN, planeImuData.angular_velocity.y()*RADIAN, planeImuData.angular_velocity.z()*RADIAN,
				planeImuData.linear_acceleration.x()*RADIAN,planeImuData.linear_acceleration.y()*RADIAN, planeImuData.linear_acceleration.z()*RADIAN,
				planeMagnetometerData.magnetic_field_body.x()*RADIAN, planeMagnetometerData.magnetic_field_body.y()*RADIAN,planeMagnetometerData.magnetic_field_body.z()*RADIAN);//�ɼ�
			//////��̬����
			//pitch_control = keep_pitch((float)x_control*0.5, (float)Drone.pitch*57.3, (float)0.1, (float)0.00 , (float)0.00000 );
			//roll_control = keep_roll((float)y_control, (float)Drone.roll, (float)0.08, (float)0.00001, (float)0.00001);
			//yaw_control = keep_yaw((float)yaw_expect, (float)Drone.yaw, (float)1, (float)0.001, (float)0.001);
			//std::cout<<"ƫ��"<<yaw_control<<"����"<< yaw_expect<<"yaw"<<Drone.yaw<<std::endl;
			//�߶ȿ���	
			high_control = keep_height(Posistion_expect.height, pos_filter.height, (float)0.006, (float)0.0000, (float)0.1);
			RpcLibClientBase.moveByAngleThrottle(x_control, y_control, high_control, 0, (float)0.04);



			//��ʾ����������
			{
				//std::cout << "�߶ȿ�����:" << high_control << "�߶�����ֵ:" << Posistion_expect.height << "��ǰ��ѹ�Ƹ߶�:" << 
				//	pos_filter.height <<  std::endl;
				/*std::cout << "GPS�߶�" << Gpsposition.altitude << std::endl;*/


				std::cout << "x������:" << x_control << "x����ֵ:" << Posistion_expect.x << "��ǰx :" <<
					pos.x << "x�˲�ֵ��" << pos_filter.x << std::endl;
				//std::cout << "pitch_control:" << pitch_control << "Drone.pitch:" << Drone.pitch * 100 << std::endl;



		 	   std::cout << "y������:" << y_control << "y����ֵ:" << Posistion_expect.y << "��ǰy:" <<
					pos.y << "y�˲�ֵ��" << pos_filter.y << std::endl; 

			   std::cout << "�߶ȿ�����:" << high_control << "�߶�����ֵ:" << Posistion_expect.height << "��ǰ�߶�:" <<
				   pos.height << "�߶��˲�ֵ��" << pos_filter.height << std::endl;
			//	std::cout << "roll_control:" << roll_control << "Drone.roll:" << Drone.roll << std::endl;
			}


			//��ȡͼ��
			//if (frame % 5 == 0)
			//{
			//	//��ȡǰ������ͷǰ��ͼ
			//	get_image(RpcLibClientBase.simGetImages(request0), path, Front_Scence_image);
			//	get_image(RpcLibClientBase.simGetImages(request1), path, Below_Scence_image);
			//	Mat Front_Scence_image1;
			//	Mat Below_Scence_image1;
			//	search_and_land(number1, Below_Scence_image, Below_Scence_image1,sign,pos_init.x,pos_init.y,pos_init.height,
			//		move.x, move.y, move.height,  Posistion_expect.x, Posistion_expect.y, Posistion_expect.height, w1, w2, b1, b2);
			//	////ʶ��
			//	//Rect number_rect;
			//	//Mat Front_Scence_image1;
			//	//int recongtion_success = 0;
			//	//Number_recognition(0, Front_Scence_image, Front_Scence_image1, number1, number_rect, w1, w2, b1, b2, recongtion_success);
			//	//if ((recongtion_success == 1) && (sign == 0))
			//	//{
			//	//	int offset_y = floor(number_rect.x + number_rect.width / 2) - 320;
			//	//	move.y = (float)0.03*offset_y;
			//	//	Posistion_expect.y = pos_init.y + move.y;
			//	//	int offset_height = floor(number_rect.y + number_rect.height / 2) - 240;
			//	//	move.height = (float)0.03*offset_height;
			//	//	Posistion_expect.height = pos_init.height + move.height;
			//	//}
			//	//Mat Below_Scence_image1;
			//	//Rect number_rect1;
			//	//int recongtion_success1 = 0;
			//	//Number_recognition(1, Below_Scence_image, Below_Scence_image1, number1, number_rect1, w1, w2, b1, b2, recongtion_success1);
			//	//if ((recongtion_success1 == 1) && (sign == 0))
			//	//{
			//	//	int offset_y = floor(number_rect1.x + number_rect1.width / 2) - 320;
			//	//	move.y = (float)0.03*offset_y;
			//	//	Posistion_expect.y = pos_init.y + move.y;
			//	//	sign = 1;
			//	//	int offset_x = floor(number_rect1.y + number_rect1.height / 2) - 240;
			//	//	move.x = (float)0.03*offset_x;
			//	//	Posistion_expect.x = pos_init.x + move.x;
			//	//}
			// 	imshow("ǰ��ǰ��ͼ", Front_Scence_image);
			// 	imshow("����ǰ��ͼ", Below_Scence_image1);
			// 	waitKey(1);
			// }




			if ((x_control < 0.02) && (y_control < 0.02)&&(sign==1))
			{
				Posistion_expect.height = -1;
			}
			if ((Gpsposition.altitude < 9) && (sign == 1))
			{
				Posistion_expect.height = pos_init.height + move.height;
				pos_init.x = pos_filter.x;
				pos_init.y = pos_filter.y;
				number1=number1+1;
				sign = 0;
			}

	 



			//	//if (response0.size() > 0)
			//	//{
			//	//	for (const ImageResponse& image_info : response0) {
			//	//		std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;//����ͼ
			//	//		std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
			//	//		if (path != "")
			//	//		{
			//	//			//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
			//	//			std::string file_path = path;
			//	//			if (image_info.pixels_as_float)
			//	//			{
			//	//				/*std::ofstream file(file_path + "3" + ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
			//	//				file.close();*/
			//	//				Mat img3 = Mat(480, 640, CV_32FC1);
			//	//				memcpy(img3.data, image_info.image_data_float.data(), image_info.image_data_float.size() * sizeof(float));
			//	//			    img3 = 1-img3 / 255;
			//	//				cv::imshow("img3", img3);
			//	//				//std::cout << img3 << std::endl;
			//	//				cv::waitKey(1);
			//	//			}
			//	//			else
			//	//			{
			//	//		/*		std::ofstream file(file_path + "1" + ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
			//	//				file.close();
			//	//				Mat img = imread(file_path + "1" + ".png");
			//	//				cv::imshow("img1",img);
			//	//				cv::waitKey(1);*/
			//	//				file_path = "F:/AirSim-master6.26/image1/";
			//	//				std::ofstream file(file_path + to_string(1)+ ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
			//	//				file.close();
			//	//				//Mat img1 = Mat(480,640,CV_8UC3);
			//	//				//memcpy(img1.data, image_info.image_data_uint8.data(), image_info.image_data_uint8.size() * sizeof(UINT8));
			//	//				Mat img = imread(file_path + to_string(1) + ".png");
			//	//				int number_rec = 0;
			//	//				Mat out; Rect img_rect;
			//	//				Number_ROI3(0, img, out, img_rect);
			//	//				number_rec = recognition(out, w1, w2, b1, b2);
			//	//				putText(img, to_string(number_rec) + "   " + to_string(img_rect.x) + "   " + to_string(img_rect.y) + "   "
			//	//					+ to_string(img_rect.width) + "   " + to_string(img_rect.height) + "   ",
			//	//					Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4, 8);//��ͼƬ��д����
			//	//				cv::imshow("ǰ������ͷǰ��ͼ", img);
			//	//				cv::waitKey(1);
			//	//			}
			//	//		}
			//	//	}
			//	//}
			//   // const std::vector<ImageResponse>& response1 = RpcLibClientBase.simGetImages(request1);
			//	//if (response1.size() > 0)
			//	//{
			//	//	for (const ImageResponse& image_info1 : response1) {
			//	//		std::cout << "Image uint8 size: " << image_info1.image_data_uint8.size() << std::endl;//����ͼ
			//	//		std::cout << "Image float size: " << image_info1.image_data_float.size() << std::endl;
			//	//		if (path1 != "")
			//	//		{
			//	//			//std::string file_path1 = FileSystem::combine(path1, std::to_string(image_info1.time_stamp));
			//	//			std::string file_path1 = path1;
			//	//			if (image_info1.pixels_as_float)
			//	//			{
			//	//		       /*Utils::writePfmFile(image_info1.image_data_float.data(), image_info1.width, image_info1.height,
			//	//					file_path1 + ".pfm");*/
			//	//			}
			//	//			else
			//	//			{
			//	//				//std::ofstream file1(file_path1 + std::to_string(frame) + ".png", std::ios::binary);
			//	//				//file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
			//	//				//file1.close();
			//	//				//std::ofstream file1(file_path1 + to_string(frame) + ".png", std::ios::binary);
			//	//				//����ͼƬ
			//	//				std::ofstream file1(file_path1 + to_string(2) + ".png", std::ios::binary);
			//	//				file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
			//	//				file1.close();
			//	//				Mat img2 = imread(file_path1+ to_string(2) + ".png");
			//	//				cv::imshow("img2", img2);
			//	//		 
			//	//				/*std::ofstream file1(file_path1 + "2"+ ".png", std::ios::binary);
			//	//				file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
			//	//				file1.close();
			//	//				Mat img2 = imread(file_path1+ "2" + ".png");
			//	//				cv::imshow("img2", img2);*/
			//	//				//Mat img3,img4;
			//	//				//Affine_transformation(img2, img3);//����任
			//	//				//Number_ROI(img3, img4);//���50*50��С�Ŀ�
			//	//				////int number = recognition(img4, w1, w2, b1, b2);
			//	//				////cout << "����ʶ��" << number << endl;
			//	//				//cv::imshow("img4", img4);
			//	//				cv::waitKey(1);
			//	//			}
			//	//		}
			//	//	}
			//	//}
			//	//const std::vector<ImageResponse>& response2 = RpcLibClientBase.simGetImages(request2);
			//	//if (response2.size() > 0)
			//	//{
			//	//	for (const ImageResponse& image_info2 : response2) {
			//	//		std::cout << "Image uint8 size: " << image_info2.image_data_uint8.size() << std::endl;//����ͼ
			//	//		std::cout << "Image float size: " << image_info2.image_data_float.size() << std::endl;
			//	//		if (path != "")
			//	//		{
			//	//			//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
			//	//			std::string file_path = path;
			//	//			if (image_info2.pixels_as_float)
			//	//			{
			//	//				/*std::ofstream file(file_path + "3" + ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
			//	//				file.close();*/
			//	//				Mat img3 = Mat(480,640,CV_32FC1);
			//	//				memcpy(img3.data, image_info2.image_data_float.data(), image_info2.image_data_float.size() * sizeof(float));
			//	//				//img3 = img3 / 255;
			//	//				cv::imshow("img3", img3);
			//	//				//std::cout << img3 << std::endl;
			//	//				cv::waitKey(1);
			//	//			}
			//	//			else
			//	//			{
			//	//				std::ofstream file(file_path + "3" + ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
			//	//				file.close();
			//	//				Mat img3 = imread(file_path + "3" + ".png");
			//	//				cv::imshow("img3", img3);
			//	//				cv::waitKey(1);
			//	//			/*	file_path = "F:/AirSim-master6.26/image2/";
			//	//				std::ofstream file(file_path + to_string(frame) + ".png", std::ios::binary);
			//	//				file.write(reinterpret_cast<const char*>(image_info2.image_data_uint8.data()), image_info2.image_data_uint8.size());
			//	//				file.close();
			//	//				Mat img3 = imread(file_path + to_string(frame) + ".png");
			//	//				cv::imshow("img3", img3);
			//	//				cv::waitKey(1);*/
			//	//			}
			//	//		}
			//	//	}
			//	//}
			//}
			


			frame++;
			std::cout << frame << std::endl;
		}

		//����
		{
			RpcLibClientBase.hover();	//��ͣģʽ
			RpcLibClientBase.land();//����
			RpcLibClientBase.armDisarm(false);//�Ͽ�����
		}

	}
	catch (rpc::rpc_error&  e) {	//API������Ϣ
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
	}
	//std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
	return 0;
}


















//
//
//
//
//
//
//
//
//// Copyright (c) Microsoft Corporation. All rights reserved.
//// Licensed under the MIT License.
//
//#include "common/common_utils/StrictMode.hpp"
//STRICT_MODE_OFF
//#ifndef RPCLIB_MSGPACK
//#define RPCLIB_MSGPACK clmdep_msgpack
//#endif // !RPCLIB_MSGPACK
//#include "rpc/rpc_error.h"
//STRICT_MODE_ON
//
//#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
//#include "vehicles/multirotor/api/StandAloneSensors.hpp"
//#include "common/common_utils/FileSystem.hpp"
//#include <iostream>
//#include <chrono>
//#include <conio.h>
////���˻�����ģ��
//#include "PID.h"
//#include "contrl.h"
//#include "GPS_ECEF.h"
//#include "IMU1.hpp" 
////ͼ����ģ��
//#include <opencv2/highgui/highgui.hpp>  
//#include <opencv2/imgproc/imgproc.hpp>  
//#include <opencv2/core/core.hpp>   
//#include <opencv2/opencv.hpp>   
//#include "preprocess.hpp"
//#include "image_process.hpp"
//using namespace cv;
//
//#define RADIAN 57.29577913f
//
//int main()
//{
//	using namespace msr::airlib;
//	msr::airlib::MultirotorRpcLibClient RpcLibClientBase;
//	typedef ImageCaptureBase::ImageRequest ImageRequest;
//	typedef ImageCaptureBase::ImageResponse ImageResponse;
//	typedef ImageCaptureBase::ImageType ImageType;
//	typedef common_utils::FileSystem FileSystem;
//	try {
//		//����ģ��
//		//std::cout << "�س����ӻ���" << std::endl; std::cin.get();
//		RpcLibClientBase.confirmConnection();//���ӻ���
//		std::cout << "�س�ʹ�ܿ���������" << std::endl;
//		RpcLibClientBase.enableApiControl(true);//ʹ��API   
//												//���˻�����ģ��
//												//�������˻�
//		RpcLibClientBase.armDisarm(true);
//		//�����ʹ��
//
//		std::cout << "���FPVͼ��" << std::endl;
//		std::vector<ImageRequest> request0 = { ImageRequest(0, ImageType::Scene) };//ǰ��ͼ
//		std::string path = "F:\\AirSim-master6.26\\image\\";
//		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
//		std::vector<ImageRequest> request1 = { ImageRequest(3, ImageType::Scene) };//����ͼ
//		std::string path1 = "F:\\AirSim-master6.26\\image\\";
//		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
//		//	std::vector<ImageRequest> request2 = { ImageRequest(0, ImageType::DepthPerspective,true) };//ǰ�����ͼ
//		std::string path2 = "F:\\AirSim-master6.26\\image\\";
//		std::cout << " ����ͼ�񵽣�F:\\AirSim-master6.26\\image" << std::endl;
//		//���
//		{
//			//���
//			float takeoffTimeout = 1;   //��ɵȴ�ʱ��
//			RpcLibClientBase.takeoff(takeoffTimeout);
//			RpcLibClientBase.hover();   		//�л�����ͣģʽ	
//		}
//
//
//		int frame = 0;//�����֡��
//		int sign = 'q';    //���̿��Ʊ�־λ
//
//						   //���GPSλ��
//		auto Gpsposition = RpcLibClientBase.getGpsLocation();
//		Gpsposition.altitude = abs(Gpsposition.altitude);
//		std::cout << "����GPS " << std::endl;
//		//��ȡ��ѹ������  ������θ߶�  ��ѹ��
//		BarometerData planeBarometerData = RpcLibClientBase.getBarometerdata((float)0.01); //����
//		std::cout << "������ѹ�� " << std::endl;
//		//��ô���������   x,y,z
//		MagnetometerData planeMagnetometerData = RpcLibClientBase.getMagnetometerdata((float)0.01);
//		std::cout << "��ȡ��������Ϣ" << std::endl;
//		//��ùߵ�����     
//		ImuData planeImuData = RpcLibClientBase.getImudata((float)0.01);
//		std::cout << "��ȡ�ߵ�����" << std::endl;
//
//		//��ʼλ��
//		auto Gpsposition_init = Gpsposition;
//		Radius radius;
//		radius = CalcEarthRadius(Gpsposition.latitude);
//		Pos pos, pos_init;
//		pos_init = Offset(Gpsposition.latitude, Gpsposition.longitude, Gpsposition_init.latitude, Gpsposition_init.longitude, radius.r1, radius.r2);
//		pos = pos_init;
//		std::cout << "X��ʼ���꣺" << pos_init.x << "Y��ʼ���꣺" << pos_init.y << std::endl;
//		float groud = Gpsposition.altitude;
//		float high_init = groud;
//		float high_control = (float)0.56;
//		//float x_control = 0;
//		//float y_control = 0;
//		int sign2 = 1;
//		Attitude drone;
//		drone.pitch = (float)0.0; drone.roll = (float)0.0; drone.yaw = (float)0.0;
//		float pitch_expect = (float)0.0; float roll_expect = (float)0.0; float yaw_expect = (float)0.0;
//		float pitch_control = (float)0.0; 	float roll_control = (float)0.0; float yaw_control = (float)0.0;
//		Attitude Drone;
//		float x_control = (float)0.0;
//		float y_control = (float)0;
//
//
//		//ͼ��
//		Mat Front_Scence_image;
//		Mat Below_Scence_image;
//		cv::Mat w1, w2, b1, b2;
//		Load_neural_network(w1, w2, b1, b2);
//		while (1)
//		{
//			//std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
//			if (kbhit())
//			{
//				sign = getch();
//				if (sign == 'w')//��ǰ��
//				{
//					//�ƶ�:���������ȣ�����������ȣ�������(0.5����λ)��ƫ���ʣ�����ʱ��
//					//RpcLibClientBase.moveByAngleThrottle((float)-0.3, 0, (float)(high_control), 0, (float)0.2);
//					pitch_expect = (float)-0.3;
//				}
//				else if (sign == 's')//����
//				{
//					//�ƶ�:���������ȣ�����������ȣ�������(0.5����λ)��ƫ���ʣ�����ʱ��
//					pitch_expect = (float)0.3;
//				}
//				else if (sign == 'd')  //�����
//				{
//					//�ƶ�:���������ȣ�����������ȣ�������(0.5����λ)��ƫ���ʣ�����ʱ��					 
//					roll_expect = (float)0.3;
//				}
//				else if (sign == 'a') //����
//				{
//					//�ƶ�:���������ȣ�����������ȣ�������(0.5����λ)��ƫ���ʣ�����ʱ��
//					roll_expect = (float)-0.3;
//				}
//				else if (sign == 'c')//ƫ��
//				{
//					//�ƶ�:���������ȣ�����������ȣ������ţ�ƫ���ʣ�����ʱ��
//					yaw_expect = (float)0.1;
//				}
//				else if (sign == 'v')//ƫ��
//				{
//					//�ƶ�:���������ȣ�����������ȣ������ţ�ƫ���ʣ�����ʱ��
//					yaw_expect = (float)-0.1;
//				}
//				else if (sign == 'j')//���Ϸ�
//				{
//					//�ƶ�:���������ȣ�����������ȣ������ţ�ƫ���ʣ�����ʱ��
//					high_control = high_control + (float)0.1;
//					if (high_control>0.75) { high_control = (float)0.75; }
//					RpcLibClientBase.moveByAngleThrottle(0, 0, (float)high_control, 0, (float)0.2);
//				}
//				else if (sign == 'k')//���·�
//				{
//					//�ƶ�:���������ȣ�����������ȣ������ţ�ƫ���ʣ�����ʱ��
//					high_control = high_control - (float)0.1;
//					if (high_control<0.3) { high_control = (float)0.3; }
//					RpcLibClientBase.moveByAngleThrottle(0, 0, (float)high_control, 0, (float)0.2);
//				}
//				else if (sign == 'l')
//				{
//					RpcLibClientBase.moveByAngleThrottle(0, 0, 0, 0, 2);
//				}
//				else if (sign == 'b')
//				{
//					RpcLibClientBase.hover();   		//�л�����ͣģʽ	
//				}
//				//std::cout << "GPS��(" << Gpsposition << ")" << std::endl;
//				//std::cout << "��ѹ�ƣ�(" << planeBarometerData << ")" << std::endl;
//				//std::cout << "��������Ϣ��(" << planeMagnetometerData.magnetic_field_body << ")" << std::endl;
//				//std::cout << "ImuData��(" << planeImuData.angular_velocity << ")" << std::endl;
//				//fflush(stdin);
//			}
//
//			Gpsposition = RpcLibClientBase.getGpsLocation();
//			planeBarometerData = RpcLibClientBase.getBarometerdata((float)0.01);
//			planeMagnetometerData = RpcLibClientBase.getMagnetometerdata((float)0.01);
//			planeImuData = RpcLibClientBase.getImudata((float)0.01);
//			pos = Offset(Gpsposition.latitude, Gpsposition.longitude, Gpsposition_init.latitude, Gpsposition_init.longitude, radius.r1, radius.r2);
//			{
//				if (sign == 'w')//��ǰ��
//				{
//					pos_init.x = pos.x;
//				}
//				else if (sign == 's')//����
//				{
//					pos_init.x = pos.x;
//				}
//				else if (sign == 'd')  //�����
//				{
//					pos_init.y = pos.y;
//				}
//				else if (sign == 'a') //����
//				{
//					pos_init.y = pos.y;
//				}
//				else if (sign == 'j')//���Ϸ�
//				{
//					high_init = Gpsposition.altitude;
//				}
//				else if (sign == 'k')//���·�
//				{
//					high_init = Gpsposition.altitude;
//				}
//			}
//			if (sign == 'q')
//			{
//				////ˮƽ����
//				x_control = keep_horizontal_x((float)pos_init.x, (float)pos.x, (float)0.1, (float)0.0001, (float)0.001);
//				y_control = keep_horizontal_y((float)pos_init.y, (float)pos.y, (float)0.1, (float)0, (float)0);
//				//IMU
//				Drone = IMUupdate(0.05f, planeImuData.angular_velocity.x()*RADIAN, planeImuData.angular_velocity.y()*RADIAN, planeImuData.angular_velocity.z()*RADIAN,
//					planeImuData.linear_acceleration.x()*RADIAN, planeImuData.linear_acceleration.y()*RADIAN, planeImuData.linear_acceleration.z()*RADIAN,
//					planeMagnetometerData.magnetic_field_body.x()*RADIAN, planeMagnetometerData.magnetic_field_body.y()*RADIAN, planeMagnetometerData.magnetic_field_body.z()*RADIAN);//�ɼ�
//																																													  //��̬����
//				pitch_control = keep_pitch((float)x_control, (float)Drone.pitch, (float)0.2, (float)0.01, (float)0.01);
//				roll_control = keep_roll((float)y_control, (float)Drone.roll, (float)0.2, (float)0.01, (float)0.01);
//				yaw_control = keep_yaw((float)yaw_expect, (float)Drone.yaw, (float)0.2, (float)0.01, (float)0.01);
//				//�߶ȿ���	
//				high_control = keep_high(high_init, Gpsposition.altitude, (float)0.2, (float)0.001, (float)0.01);
//				//std::cout << "�߶ȿ��ƣ�" << high_control <<  std::endl;
//				RpcLibClientBase.moveByAngleThrottle((float)(-pitch_control), roll_control, high_control, 0, (float)0.02);
//			}
//			else
//			{
//				high_control = keep_high(high_init, Gpsposition.altitude, (float)0.2, (float)0.001, (float)0.45);
//				RpcLibClientBase.moveByAngleThrottle(pitch_expect, roll_expect, high_control, yaw_expect, (float)0.02);
//			}
//
//			sign = 'q';
//			pitch_expect = 0;
//			roll_expect = 0;
//			yaw_expect = 0;
//
//			{
//				if ((sign2 == 0))
//				{
//					high_init = Gpsposition.altitude;
//					sign2 = 2;
//				}
//				if ((Gpsposition.altitude<0) && (sign2 == 1))
//				{
//					sign2 = 0;
//				}
//			}
//			//��ʾ����������
//			{
//				//std::cout << "GPS ���ȣ�(" << (double)Gpsposition.longitude << ")" << std::endl;
//				//std::cout << "GPS γ�ȣ�(" << (double)Gpsposition.latitude << ")" << std::endl;
//				//std::cout << "GPS ���θ߶ȣ�(" << (double)(Gpsposition.altitude) << ")" << std::endl;
//
//				//std::cout << "��ѹ�� ѹ��ֵ��(" << planeBarometerData.pressure << ")" << std::endl;
//				//std::cout << "��ѹ�� �߶�(" << planeBarometerData.altitude << ")" << std::endl;
//
//				//std::cout << "������ x��(" << planeMagnetometerData.magnetic_field_body.x() << ")" << std::endl;
//				//std::cout << "������ y��(" << planeMagnetometerData.magnetic_field_body.y() << ")" << std::endl;
//				//std::cout << "������ z��(" << planeMagnetometerData.magnetic_field_body.z() << ")" << std::endl;
//
//				//std::cout << "ImuData ���ٶ�1��(" << planeImuData.angular_velocity.x() << ")" << std::endl;
//				//std::cout << "ImuData ���ٶ�2��(" << planeImuData.angular_velocity.y() << ")" << std::endl;
//				//std::cout << "ImuData ���ٶ�3��(" << planeImuData.angular_velocity.z() << ")" << std::endl;
//
//				//std::cout << "ImuData �Ǽ��ٶ�1 (" << planeImuData.linear_acceleration.x() << ")" << std::endl;
//				//std::cout << "ImuData �Ǽ��ٶ�2(" << planeImuData.linear_acceleration.y() << ")" << std::endl;
//				//std::cout << "ImuData �Ǽ��ٶ�3(" << planeImuData.linear_acceleration.z() << ")" << std::endl;
//
//				//std::cout << "�߶ȿ�������  " << high_control << std::endl;
//				//std::cout << "X������� ��  " << x_control << std::endl;
//				//std::cout << "Y������� ��  " << y_control << std::endl;
//
//				//std::cout << "�߶�  SET:" << high_init << " NOW:" << (Gpsposition.altitude) << " THR:" << std::endl;
//				//std::cout << "X���� SET:" << pos_init.x << " NOW:" << pos.x << " THR:" << std::endl;
//				//std::cout << "Y���� SET:" << pos_init.y << " NOW:" << pos.y << " THR:" << std::endl;
//
//			}
//
//			//��ȡͼ��
//			if (frame % 10 == 0)
//			{
//				//const std::vector<ImageResponse>& response0 = RpcLibClientBase.simGetImages(request0);
//				//��ȡǰ������ͷǰ��ͼ
//				get_image(RpcLibClientBase.simGetImages(request0), path, Front_Scence_image);
//				get_image(RpcLibClientBase.simGetImages(request1), path, Below_Scence_image);
//				//ʶ��
//				Mat number_image;
//				Rect number_rect;
//				int number = 0;
//				//Number_ROI(0,Front_Scence_image, number_image, number_rect);//��ȡROI
//				//number=recognition(number_image,  w1,  w2,  b1,  b2);//ʶ������
//				image_process(0, Front_Scence_image, number, number_rect, w1, w2, b1, b2);
//				putText(Front_Scence_image, to_string(number) + "   " + to_string(number_rect.x) + "   " + to_string(number_rect.y) + "   "
//					+ to_string(number_rect.width) + "   " + to_string(number_rect.height) + "   ",
//					Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4, 8);//��ͼƬ��д����
//
//
//				imshow("ǰ��ǰ��ͼ", Front_Scence_image);
//				imshow("����ǰ��ͼ", Below_Scence_image);
//				waitKey(1);
//				//if (response0.size() > 0)
//				//{
//				//	for (const ImageResponse& image_info : response0) {
//				//		std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;//����ͼ
//				//		std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;
//				//		if (path != "")
//				//		{
//				//			//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
//				//			std::string file_path = path;
//				//			if (image_info.pixels_as_float)
//				//			{
//				//				/*std::ofstream file(file_path + "3" + ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
//				//				file.close();*/
//				//				Mat img3 = Mat(480, 640, CV_32FC1);
//				//				memcpy(img3.data, image_info.image_data_float.data(), image_info.image_data_float.size() * sizeof(float));
//				//			    img3 = 1-img3 / 255;
//				//				cv::imshow("img3", img3);
//				//				//std::cout << img3 << std::endl;
//				//				cv::waitKey(1);
//				//			}
//				//			else
//				//			{
//				//		/*		std::ofstream file(file_path + "1" + ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
//				//				file.close();
//				//				Mat img = imread(file_path + "1" + ".png");
//				//				cv::imshow("img1",img);
//				//				cv::waitKey(1);*/
//				//				file_path = "F:/AirSim-master6.26/image1/";
//				//				std::ofstream file(file_path + to_string(1)+ ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
//				//				file.close();
//				//				//Mat img1 = Mat(480,640,CV_8UC3);
//				//				//memcpy(img1.data, image_info.image_data_uint8.data(), image_info.image_data_uint8.size() * sizeof(UINT8));
//				//				Mat img = imread(file_path + to_string(1) + ".png");
//				//				int number_rec = 0;
//				//				Mat out; Rect img_rect;
//				//				Number_ROI3(0, img, out, img_rect);
//				//				number_rec = recognition(out, w1, w2, b1, b2);
//				//				putText(img, to_string(number_rec) + "   " + to_string(img_rect.x) + "   " + to_string(img_rect.y) + "   "
//				//					+ to_string(img_rect.width) + "   " + to_string(img_rect.height) + "   ",
//				//					Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 4, 8);//��ͼƬ��д����
//				//				cv::imshow("ǰ������ͷǰ��ͼ", img);
//				//				cv::waitKey(1);
//				//			}
//				//		}
//				//	}
//				//}
//				// const std::vector<ImageResponse>& response1 = RpcLibClientBase.simGetImages(request1);
//				//if (response1.size() > 0)
//				//{
//				//	for (const ImageResponse& image_info1 : response1) {
//				//		std::cout << "Image uint8 size: " << image_info1.image_data_uint8.size() << std::endl;//����ͼ
//				//		std::cout << "Image float size: " << image_info1.image_data_float.size() << std::endl;
//				//		if (path1 != "")
//				//		{
//				//			//std::string file_path1 = FileSystem::combine(path1, std::to_string(image_info1.time_stamp));
//				//			std::string file_path1 = path1;
//				//			if (image_info1.pixels_as_float)
//				//			{
//				//		       /*Utils::writePfmFile(image_info1.image_data_float.data(), image_info1.width, image_info1.height,
//				//					file_path1 + ".pfm");*/
//				//			}
//				//			else
//				//			{
//				//				//std::ofstream file1(file_path1 + std::to_string(frame) + ".png", std::ios::binary);
//				//				//file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
//				//				//file1.close();
//				//				//std::ofstream file1(file_path1 + to_string(frame) + ".png", std::ios::binary);
//				//				//����ͼƬ
//				//				std::ofstream file1(file_path1 + to_string(2) + ".png", std::ios::binary);
//				//				file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
//				//				file1.close();
//				//				Mat img2 = imread(file_path1+ to_string(2) + ".png");
//				//				cv::imshow("img2", img2);
//				//		 
//				//				/*std::ofstream file1(file_path1 + "2"+ ".png", std::ios::binary);
//				//				file1.write(reinterpret_cast<const char*>(image_info1.image_data_uint8.data()), image_info1.image_data_uint8.size());
//				//				file1.close();
//				//				Mat img2 = imread(file_path1+ "2" + ".png");
//				//				cv::imshow("img2", img2);*/
//				//				//Mat img3,img4;
//				//				//Affine_transformation(img2, img3);//����任
//				//				//Number_ROI(img3, img4);//���50*50��С�Ŀ�
//				//				////int number = recognition(img4, w1, w2, b1, b2);
//				//				////cout << "����ʶ��" << number << endl;
//				//				//cv::imshow("img4", img4);
//				//				cv::waitKey(1);
//				//			}
//				//		}
//				//	}
//				//}
//				//const std::vector<ImageResponse>& response2 = RpcLibClientBase.simGetImages(request2);
//				//if (response2.size() > 0)
//				//{
//				//	for (const ImageResponse& image_info2 : response2) {
//				//		std::cout << "Image uint8 size: " << image_info2.image_data_uint8.size() << std::endl;//����ͼ
//				//		std::cout << "Image float size: " << image_info2.image_data_float.size() << std::endl;
//				//		if (path != "")
//				//		{
//				//			//std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
//				//			std::string file_path = path;
//				//			if (image_info2.pixels_as_float)
//				//			{
//				//				/*std::ofstream file(file_path + "3" + ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
//				//				file.close();*/
//				//				Mat img3 = Mat(480,640,CV_32FC1);
//				//				memcpy(img3.data, image_info2.image_data_float.data(), image_info2.image_data_float.size() * sizeof(float));
//				//				//img3 = img3 / 255;
//				//				cv::imshow("img3", img3);
//				//				//std::cout << img3 << std::endl;
//				//				cv::waitKey(1);
//				//			}
//				//			else
//				//			{
//				//				std::ofstream file(file_path + "3" + ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info2.image_data_float.data()), image_info2.image_data_float.size());
//				//				file.close();
//				//				Mat img3 = imread(file_path + "3" + ".png");
//				//				cv::imshow("img3", img3);
//				//				cv::waitKey(1);
//				//			/*	file_path = "F:/AirSim-master6.26/image2/";
//				//				std::ofstream file(file_path + to_string(frame) + ".png", std::ios::binary);
//				//				file.write(reinterpret_cast<const char*>(image_info2.image_data_uint8.data()), image_info2.image_data_uint8.size());
//				//				file.close();
//				//				Mat img3 = imread(file_path + to_string(frame) + ".png");
//				//				cv::imshow("img3", img3);
//				//				cv::waitKey(1);*/
//				//			}
//				//		}
//				//	}
//				//}
//			}
//
//
//			frame++;
//			std::cout << frame << std::endl;
//		}
//
//		//����
//		{
//			RpcLibClientBase.hover();	//��ͣģʽ
//			RpcLibClientBase.land();//����
//			RpcLibClientBase.armDisarm(false);//�Ͽ�����
//		}
//
//	}
//	catch (rpc::rpc_error&  e) {	//API������Ϣ
//		std::string msg = e.get_error().as<std::string>();
//		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
//	}
//	//std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
//	return 0;
//}
