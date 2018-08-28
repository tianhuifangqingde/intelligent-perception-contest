#include <QApplication>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <process.hpp>
#include <iostream>
#include <fstream>
#include "serial/serial.h"
#include <sys/time.h>
serial::Serial m_serial;
Rect img_rect1;
int number_expect1 =6;
int success1=0;
int re_number1=0;


Rect img_rect2;
int number_expect2 =6;
int success2=0;
int re_number2=0;
void format_data_to_send1(void)
{
    unsigned char data_to_send[50];
    int Length = 0;
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x61;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = int(img_rect1.x+img_rect1.width/2)>>8;
    data_to_send[_cnt++] = int(img_rect1.x+img_rect1.width/2)%256;
    data_to_send[_cnt++] = int(img_rect1.y+img_rect1.height/2)>>8;
    data_to_send[_cnt++] = int(img_rect1.y+img_rect1.height/2)%256;
    data_to_send[_cnt++] = int(img_rect1.width)>>8;
    data_to_send[_cnt++] = int(img_rect1.width)%256;
    data_to_send[_cnt++] = int(img_rect1.height)>>8;
    data_to_send[_cnt++] = int(img_rect1.height)%256;
    data_to_send[_cnt++] = int(number_expect1)>>8;
    data_to_send[_cnt++] = int(number_expect1)%256;
    data_to_send[_cnt++] = int(success1)%256;
    data_to_send[_cnt++] = int(re_number1)%256;
    data_to_send[3] = _cnt - 4;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    Length = _cnt;
   m_serial.write((unsigned char *)data_to_send, Length);

}
void format_data_to_send2(void)
{
    unsigned char data_to_send[50];
    int Length = 0;
    int _cnt = 0, i = 0, sum = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = 0xAF;
    data_to_send[_cnt++] = 0x62;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = int(img_rect2.x+img_rect2.width/2)>>8;
    data_to_send[_cnt++] = int(img_rect2.x+img_rect2.width/2)%256;
    data_to_send[_cnt++] = int(img_rect2.y+img_rect2.height/2)>>8;
    data_to_send[_cnt++] = int(img_rect2.y+img_rect2.height/2)%256;
    data_to_send[_cnt++] = int(img_rect2.width)>>8;
    data_to_send[_cnt++] = int(img_rect2.width)%256;
    data_to_send[_cnt++] = int(img_rect2.height)>>8;
    data_to_send[_cnt++] = int(img_rect2.height)%256;
    data_to_send[_cnt++] = int(number_expect2)>>8;
    data_to_send[_cnt++] = int(number_expect2)%256;
    data_to_send[_cnt++] = int(success2)%256;
    data_to_send[_cnt++] = int(re_number2)%256;
    data_to_send[3] = _cnt - 4;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    Length = _cnt;
   m_serial.write((unsigned char *)data_to_send, Length);

}

int main()
{
   VideoCapture cap1;
    cap1.open(0);
    if(!cap1.isOpened())
    {return 0;}


    VideoCapture cap2;
    cap2.open(1);
    if(!cap2.isOpened())
    {return 0;}


     std::string port_name;
     int baudrate = 115200;
     serial::Timeout timeout(serial::Timeout::simpleTimeout(1000));
     m_serial.setPort("/dev/ttySAC0");
     m_serial.setBaudrate(baudrate);
     m_serial.setTimeout(timeout);
     m_serial.open();
     if (!m_serial.isOpen())
     {
       cout << "serial open failed!" << endl;
       return -1;
     }
     Mat frame,frame1,frame21,frame22;
     tesseract_init();
     int frame_number=0;

     Mat w1, w2, b1, b2;
     Load_neural_network(w1, w2, b1, b2);


     //struct timeval _tstart, _tend;

    // VideoWriter writer1("/home/odroid/workspace/QT/beifeng/before.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(240, 180));
    // VideoWriter writer2("/home/odroid/workspace/QT/beifeng/below.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(240, 180));



      //VideoCapture caa;
      //caa.open("/home/odroid/workspace/QT/beifeng/below8.avi");
       while(1)
       {
         cap1>>frame;
         resize(frame,frame,Size(240,180));
         //writer1<<frame;
         if(frame.empty())
         break;
         Number_recognition1(1,frame,frame1,number_expect1,img_rect1,
            success1,re_number1,w1,w2,b1,b2);
         cout<<" re_number1   "<< re_number1<<endl;
         format_data_to_send1();


         cap2>>frame21;
         flip(frame21,frame21,-1);
         resize(frame21,frame21,Size(240,180));
         //writer2<<frame21;

         if(frame21.empty())
         break;
         Number_recognition2(1,frame21,frame22,number_expect2,img_rect2,
                             success2,re_number2,w1,w2,b1,b2);
         cout<<" re_number2   "<< re_number2<<endl;
         format_data_to_send2();

         imshow("cap1",frame1);
         imshow("cap2",frame22);


         success1=0;re_number1=0;
         success2=0;re_number2=0;

         frame_number++;
         if(waitKey(1)=='a')
         {
             cap1.release();
             cap2.release();
             break;
         }
      }
      waitKey();
      return 0;
}

