#ifndef PROCESS_HPP
#define PROCESS_HPP


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "tesseract/baseapi.h"
#define LEN(array,len){len=sizeof(array)/sizeof(array[0]);}
using namespace cv;
using namespace std;

tesseract::TessBaseAPI  tess;
void tesseract_init()
{
    tess.Init(NULL, "eng");
    tess.SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
}


void Load_neural_network(OutputArray w1, OutputArray w2, OutputArray b1, OutputArray b2)
{
    FileStorage fs("/home/odroid/workspace/QT/exercise2/w1.xml", FileStorage::READ);
    Mat w11;
    fs["w1"] >> w11;
    transpose(w11, w1);
    fs.release();
    //w2
    FileStorage fs2("/home/odroid/workspace/QT/exercise2/w2.xml", FileStorage::READ);
    Mat  w3;
    fs2["w2"] >> w3;
    w3 = w3.reshape(0, 25);
    w3.copyTo(w2);
    fs2.release();
    //b1
    Mat b11;
    FileStorage fs3("/home/odroid/workspace/QT/exercise2/b1.xml", FileStorage::READ);
    fs3["b1"] >> b11;
    b11.copyTo(b1);
    fs3.release();
    //b2
    FileStorage fs4("/home/odroid/workspace/QT/exercise2/b2.xml", FileStorage::READ);
    Mat b21;
    fs4["b2"] >> b21;
    b21.copyTo(b2);
    fs4.release();
}
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
            image_feature1[(j - 1) * 5 + k - 1] = (500.0 - sum) / 500.0;
        }
    }
    Mat image_feature = Mat(25, 1, CV_32FC1, image_feature1);
    Mat hidden, hiddenout, val, out;
    float  hiddenout_s[25] = { 0 };
    hidden = w1*image_feature + b1;
    for (int m = 0; m < 25; m++)
    {

        float a = -hidden.at<float>(m, 0);
        a = 1 / (1 + exp(a));
        hiddenout_s[m] = a;
    }
    hiddenout = Mat(1, 25, CV_32FC1, hiddenout_s);
    transpose(hiddenout*w2, val);
    out = val + b2;
    float max = -10.0;
    float number = 10;
    for (int i = 0; i < 10; i++)
    {
        if (out.at<float>(i) > max)
        {
            max = out.at<float>(i);
            number = i;
        }
    }
    return number;
}


void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/8;
    double T = 0.3;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 0;
            else
                p_outputMat[j] = 255;
        }
    }
}




void rgb_hsv_two1(Mat img,OutputArray img2)
{
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,imgHSV);

    inRange(imgHSV, Scalar(15, 135, 90), Scalar(28, 255, 255), img2); //Threshold the image
    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(img2, img2, MORPH_OPEN, element);
     //闭操作 (连接一些连通域)
    morphologyEx(img2, img2, MORPH_CLOSE, element);

}


void rgb_hsv_two2(Mat img,OutputArray img2)
{
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    //因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,imgHSV);


   // inRange(imgHSV, Scalar(18, 40, 160), Scalar(43, 255, 255), img2);
    inRange(imgHSV, Scalar(20, 40, 190), Scalar(50, 110, 255), img2); //Threshold the image
    //night inRange(imgHSV, Scalar(18, 40, 160), Scalar(43, 255, 255), img2);
    //开操作 (去除一些噪点)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(img2, img2, MORPH_OPEN, element);
     //闭操作 (连接一些连通域)
    morphologyEx(img2, img2, MORPH_CLOSE, element);

}



 int MaxrepeatingNum(int aa[],int n)
 {
    int a[n];
    for(int i=0;i<n;i++)
    {
        a[i]=aa[i];
    }

    int k,maxk=0,max=a[0];
    for(int i=0;i<n;i++)
    {
        k=0;
        for(int j=0;j<i;j++)
        {
            k++;
            if(k>maxk)
            {
                maxk=k;
                max=a[j];
            }
        }
    }
     return max;
 }



void Number_recognition1(int camera_id, Mat img, OutputArray out, int number_expect,
  Rect &img_rect, int &success,int &re_number,Mat &w1,Mat &w2,Mat &b1,Mat &b2)
  {
      int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
      int number=0,number1=0;
      Mat binImg,srcImg2;
      Mat srcImg;
      Rect target_boundRect;
      CvPoint2D32f rectpoint[4];
      Mat dstImg;
      Mat RatationedImg(img.rows, img.cols, CV_8UC1);
      img.copyTo(srcImg2);
      rgb_hsv_two1(img,srcImg);
      imshow("cap1erzhihua",srcImg);
      srcImg.copyTo(binImg);
      vector<vector<Point> > contours;
      vector<Rect> boundRect(contours.size());
      //注意第5个参数为CV_RETR_EXTERNAL，只检索外框
      findContours(binImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找轮廓
      for (int i = 0; i < contours.size(); i++)
      {
          //需要获取的坐标
          CvBox2D rect =minAreaRect(Mat(contours[i]));
          cvBoxPoints(rect, rectpoint); //获取4个顶点坐标
          //与水平线的角度
          float angle = 0;
          int line1 = sqrt((rectpoint[1].y - rectpoint[0].y)*(rectpoint[1].y - rectpoint[0].y) + (rectpoint[1].x - rectpoint[0].x)*(rectpoint[1].x - rectpoint[0].x));
          int line2 = sqrt((rectpoint[3].y - rectpoint[0].y)*(rectpoint[3].y - rectpoint[0].y) + (rectpoint[3].x - rectpoint[0].x)*(rectpoint[3].x - rectpoint[0].x));
          //rectangle(binImg, rectpoint[0], rectpoint[3], Scalar(255), 2);
          //面积太小的直接pass
          if (line1 * line2 < 600)
          {
              continue;
          }
          sign[0] = i;
          for(int j=0;j<4;j++)
          {
              line(img,rectpoint[j],rectpoint[(j+1)%4],Scalar(255,0,0),2,8);
          }
          //为了让正方形横着放，所以旋转角度是不一样的。竖放的，给他加90度，翻过来

          //新建一个感兴趣的区域图，大小跟原图一样大
          Mat RoiSrcImg(srcImg.rows, srcImg.cols, CV_8UC3); //注意这里必须选CV_8UC3
          RoiSrcImg.setTo(0); //颜色都设置为黑色
          drawContours(binImg, contours, -1, Scalar(255),CV_FILLED);

          //抠图到RoiSrcImg
          srcImg.copyTo(RoiSrcImg, binImg);

          //创建一个旋转后的图像
          RatationedImg.setTo(0);
          //对RoiSrcImg进行旋转
          Point2f center = rect.center;  //中心点
          Mat M2 = getRotationMatrix2D(center, angle, 1);//计算旋转加缩放的变换矩阵
          warpAffine(RoiSrcImg, RatationedImg, M2, RoiSrcImg.size(),1, 0, Scalar(0));//仿射变换
          target_boundRect=minAreaRect(Mat(contours[sign[0]])).boundingRect();
      }
      if(contours.size()!=0)
      {
          vector<vector<Point> > contours2;
          Mat SecondFindImg;
          RatationedImg.copyTo(SecondFindImg);
          //imshow("SecondFindImg",SecondFindImg);
          //SecondFindImg.setTo(0);

          findContours(SecondFindImg, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
          //cout << "sec contour:" << contours2.size() << endl;

          for (int j = 0; j < contours2.size(); j++)
          {
              //这时候其实就是一个长方形了，所以获取rect
              Rect rect = boundingRect(Mat(contours2[j]));
              //面积太小的轮廓直接pass,通过设置过滤面积大小，可以保证只拿到外框
              if (rect.area() < 600)
              {
                  continue;
              }

              rect.x= rect.x+rect.width*0.1;
              rect.y = rect.y +rect.height*0.1;
              rect.width = rect.width*0.8;
              rect.height = rect.height*0.8;
              if (rect.x <= 0) { rect.x = 1; }	if (rect.x >= img.cols-1) { rect.x = img.cols - 1; }
              if (rect.y <= 0) { rect.y = 1; }	if (rect.y >= img.rows - 1) { rect.y = img.rows - 1; }
              if (rect.x + rect.width > (img.cols - 1)) { rect.width = img.cols - 1 - rect.x; }
              if (rect.y + rect.height > ( img.rows - 1)) { rect.height = img.rows - 1 - rect.y; }
              dstImg = RatationedImg(rect);
              imshow("before", dstImg);
          }
          if((dstImg.cols>0)&&(dstImg.rows>0))
          {
              resize(dstImg,dstImg,Size(50,50));
              number1 = recognition(dstImg,w1,w2,b1,b2);
              if(number1!=0)
              {
                 number=number1;
              }

              if ( number!=0)
               {
                   img_rect = target_boundRect;
                   success = 1;
                   re_number=number;
                   for(int j=0;j<4;j++)
                   {
                       line(srcImg2,rectpoint[j],rectpoint[(j+1)%4],Scalar(0,255,0),2,8);
                   }
                   cv::putText(srcImg2, to_string(number) ,Point((rectpoint[2].x), (rectpoint[2].y))
                      , CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 2, 4);
               }
          }
      }
      else
      {
          re_number=0;
      }
      if( re_number==0)
      {
          img_rect.x=0; img_rect.y=0; img_rect.width=0; img_rect.height=0;
          success = 0;
      }
      srcImg2.copyTo(out);
  }


void Number_recognition2(int camera_id, Mat img, OutputArray out, int number_expect,
 Rect &img_rect, int &success,int &re_number,Mat &w1,Mat &w2,Mat &b1,Mat &b2)
{
     int sign[10] = { -1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
     int number=0,number1=0;
     Mat binImg,srcImg2,img1;
     Mat srcImg;

     Rect target_boundRect;
     CvPoint2D32f rectpoint[4];
     Mat dstImg2;
     Mat RatationedImg(img.rows, img.cols, CV_8UC1);
     img.copyTo(srcImg2);
     rgb_hsv_two2(img,srcImg);

    // cvtColor(img, img1, CV_BGR2GRAY);
    // threshold(img1,srcImg,86,255,THRESH_BINARY_INV|THRESH_OTSU);

    // srcImg=255-srcImg;

     imshow("cap2yuaerzhihua",srcImg);
     srcImg.copyTo(binImg);

     vector<vector<Point> > contours;
     vector<Rect> boundRect(contours.size());
     //注意第5个参数为CV_RETR_EXTERNAL，只检索外框
     findContours(binImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE); //找轮廓
     //cout << contours.size() << endl;
     for (int i = 0; i < contours.size(); i++)
     {
         //需要获取的坐标
         CvBox2D rect =minAreaRect(Mat(contours[i]));
         cvBoxPoints(rect, rectpoint); //获取4个顶点坐标
         //与水平线的角度
         float angle = rect.angle;
         int line1 = sqrt((rectpoint[1].y - rectpoint[0].y)*(rectpoint[1].y - rectpoint[0].y) + (rectpoint[1].x - rectpoint[0].x)*(rectpoint[1].x - rectpoint[0].x));
         int line2 = sqrt((rectpoint[3].y - rectpoint[0].y)*(rectpoint[3].y - rectpoint[0].y) + (rectpoint[3].x - rectpoint[0].x)*(rectpoint[3].x - rectpoint[0].x));
         //rectangle(binImg, rectpoint[0], rectpoint[3], Scalar(255), 2);
         //面积太小的直接pass
         if (line1 * line2 < 600)
         {
             continue;
         }
         sign[0] = i;
         for(int j=0;j<4;j++)
         {
             line(img,rectpoint[j],rectpoint[(j+1)%4],Scalar(255,0,0),2,8);
         }

         //为了让正方形横着放，所以旋转角度是不一样的。竖放的，给他加90度，翻过来
         if (line1 > line2)
         {
             angle = 90 + angle;
         }
         //新建一个感兴趣的区域图，大小跟原图一样大
         Mat RoiSrcImg(srcImg.rows, srcImg.cols, CV_8UC3); //注意这里必须选CV_8UC3
         RoiSrcImg.setTo(0); //颜色都设置为黑色
         //imshow("新建的ROI", RoiSrcImg);
         //对得到的轮廓填充一下
         drawContours(binImg, contours, -1, Scalar(255),CV_FILLED);

         //抠图到RoiSrcImg
         srcImg.copyTo(RoiSrcImg, binImg);

         //创建一个旋转后的图像
         RatationedImg.setTo(0);
         //对RoiSrcImg进行旋转
         Point2f center = rect.center;  //中心点
         Mat M2 = getRotationMatrix2D(center, angle, 1);//计算旋转加缩放的变换矩阵
         warpAffine(RoiSrcImg, RatationedImg, M2, RoiSrcImg.size(),1, 0, Scalar(0));//仿射变换

         target_boundRect=minAreaRect(Mat(contours[sign[0]])).boundingRect();
     }
     if(contours.size()!=0)
     {
         vector<vector<Point> > contours2;
         Mat SecondFindImg2;
         RatationedImg.copyTo(SecondFindImg2);

         findContours(SecondFindImg2, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

         for (int j = 0; j < contours2.size(); j++)
         {
             //这时候其实就是一个长方形了，所以获取rect
             Rect rect = boundingRect(Mat(contours2[j]));
             //面积太小的轮廓直接pass,通过设置过滤面积大小，可以保证只拿到外框
             if (rect.area() < 600)
             {
                 continue;
             }

             rect.x= rect.x+rect.width*0.1;
             rect.y = rect.y +rect.height*0.1;
             rect.width = rect.width*0.8;
             rect.height = rect.height*0.8;
             if (rect.x <= 0) { rect.x = 1; }	if (rect.x >= img.cols-1) { rect.x = img.cols - 1; }
             if (rect.y <= 0) { rect.y = 1; }	if (rect.y >= img.rows - 1) { rect.y = img.rows - 1; }
             if (rect.x + rect.width > (img.cols - 1)) { rect.width = img.cols - 1 - rect.x; }
             if (rect.y + rect.height > ( img.rows - 1)) { rect.height = img.rows - 1 - rect.y; }
             dstImg2 = RatationedImg(rect);
             imshow("below", dstImg2);
         }
         if((dstImg2.cols>0)&&(dstImg2.rows>0))
         {
             resize(dstImg2,dstImg2,Size(50,50));
             number1 = recognition(dstImg2,w1,w2,b1,b2);
             if(number1!=0)
              {
               number=number1;
              }
             if ( number!=0)
              {
                  img_rect = target_boundRect;
                  success = 1;
                  re_number=number;
                  for(int j=0;j<4;j++)
                  {
                      line(srcImg2,rectpoint[j],rectpoint[(j+1)%4],Scalar(0,255,0),2,8);
                  }
                  cv::putText(srcImg2, to_string(number) ,Point((rectpoint[2].x), (rectpoint[2].y))
                     , CV_FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 0), 2, 4);
              }
         }
     }
     else
     {
         re_number=0;
     }
     if( re_number==0)
     {
         img_rect.x=0; img_rect.y=0; img_rect.width=0; img_rect.height=0;
         success = 0;
     }
     srcImg2.copyTo(out);
 }



#endif // PROCESS_HPP
