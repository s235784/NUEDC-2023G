#include <ros/ros.h>
#include <opencv2/opencv.hpp>
  
using namespace cv;  
  
int main(int argc,char *argv[])  
{
    ros::init(argc, argv, "utils_code_node");
	ros::NodeHandle nh;
    ros::Rate rate(30.0);
    
    ROS_INFO("OpenCV Version: " CV_VERSION);
    ROS_INFO("OpenCV Cuda: %d", cv::cuda::getCudaEnabledDeviceCount());

    Mat image,imageGray,imageGuussian;  
    Mat imageSobelX,imageSobelY,imageSobelOut;
    // VideoCapture cap;
    // cap.open("/dev/usbcam0");

    //VideoCapture capture("v4l2src device=/dev/usbcam0 ! video/x-raw,format=YUYV,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1", CAP_GSTREAMER);
    VideoCapture capture("/dev/usbcam0");
    if (!capture.isOpened()) {
       ROS_ERROR("Can not open camera!");
       return -1;
    }

    while (ros::ok()) {
        capture.read(image);
        //1. 原图像大小调整，提高运算效率  
    resize(image,image,Size(500,300));  
    imshow("1.原图像",image);  
  
    //2. 转化为灰度图  
    cvtColor(image,imageGray,cv::COLOR_RGB2GRAY);
    //imshow("2.灰度图",imageGray);  
  
    //3. 高斯平滑滤波  
    GaussianBlur(imageGray,imageGuussian,Size(3,3),0);  
    //imshow("3.高斯平衡滤波",imageGuussian);
  
    //4.求得水平和垂直方向灰度图像的梯度差,使用Sobel算子  
    Mat imageX16S,imageY16S;  
    Sobel(imageGuussian,imageX16S,CV_16S,1,0,3,1,0,4);  
    Sobel(imageGuussian,imageY16S,CV_16S,0,1,3,1,0,4);  
    convertScaleAbs(imageX16S,imageSobelX,1,0);  
    convertScaleAbs(imageY16S,imageSobelY,1,0);  
    imageSobelOut=imageSobelX-imageSobelY;
    // imshow("4.X方向梯度",imageSobelX);  
    // imshow("4.Y方向梯度",imageSobelY);  
    // imshow("4.XY方向梯度差",imageSobelOut);    
  
    //5.均值滤波，消除高频噪声  
    blur(imageSobelOut,imageSobelOut,Size(3,3));  
    //imshow("5.均值滤波",imageSobelOut);   
  
    //6.二值化  
    Mat imageSobleOutThreshold;  
    threshold(imageSobelOut,imageSobleOutThreshold,180,255,cv::THRESH_BINARY);
    imshow("6.二值化",imageSobleOutThreshold);  
  
    //7.闭运算，填充条形码间隙  
    Mat  element=getStructuringElement(0,Size(7,7));  
    morphologyEx(imageSobleOutThreshold,imageSobleOutThreshold,MORPH_CLOSE,element);      
    // imshow("7.闭运算",imageSobleOutThreshold);  
  
    //8. 腐蚀，去除孤立的点  
    erode(imageSobleOutThreshold,imageSobleOutThreshold,element);  
    // imshow("8.腐蚀",imageSobleOutThreshold);  
  
    //9. 膨胀，填充条形码间空隙，根据核的大小，有可能需要2~3次膨胀操作  
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);  
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);  
    dilate(imageSobleOutThreshold,imageSobleOutThreshold,element);  
    // imshow("9.膨胀",imageSobleOutThreshold);        
    std::vector<std::vector<Point>> contours;  
    std::vector<Vec4i> hiera;
  
    //10.通过findContours找到条形码区域的矩形边界  
    findContours(imageSobleOutThreshold,contours,hiera,RETR_EXTERNAL,CHAIN_APPROX_NONE);
    for(int i=0;i<contours.size();i++)
    {  
        Rect rect=boundingRect((Mat)contours[i]);  
        rectangle(image,rect,Scalar(255),2);      
    }
    imshow("10.找出二维码矩形区域",image);
    waitKey(1);
    }

    capture.release();
    destroyAllWindows();
}