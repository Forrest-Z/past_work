#include <iostream>
#include<opencv2/opencv.hpp>

 using namespace cv;

 int main()
 {
     //1.从摄像头读入视频
     VideoCapture cap(1);
     //2.循环显示每一帧
     while (1)
     {
         Mat cam;
         cap >> cam;//获取当前帧图像
         imshow("相机",cam);//显示当前帧图像
         waitKey(30);//延时30秒
     }
     return 0;
 }