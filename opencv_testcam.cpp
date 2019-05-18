#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<vector>
#include<iostream> 
#include<geometry_msgs/Twist.h>
#include "std_msgs/String.h"
static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;
class ImageConverter
{
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
image_transport::Subscriber depth_sub_;

ros::Publisher cmdVelPub;
ros::Publisher isDetect;
geometry_msgs::Twist speed;

public:
//定义全局变量
float average_x,average_y;
int count_circle;
const int X_MIDDLE = 240;
ImageConverter()
: it_(nh_)
{
// Subscrive to input video feed and publish output video feed
image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
&ImageConverter::imageCb, this);
image_pub_ = it_.advertise("/image_converter/output_video", 1);
depth_sub_ = it_.subscribe("/camera/depth/image_raw",1,&ImageConverter::depthCb,this);
cmdVelPub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
isDetect = nh_.advertise<std_msgs::String>("chatter",10);


 
// go around the map and avoid the obstacles

}
 
~ImageConverter()
{
cv::destroyWindow(OPENCV_WINDOW);
}
 
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
cv_bridge::CvImagePtr cv_ptr;
try
{
cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}
catch (cv_bridge::Exception& e)
{
ROS_ERROR("cv_bridge exception: %s", e.what());
return;
}
 
// Draw an example circle on the video stream
//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
 
//RGB to Gray
//cv::Mat imgGray;
//cvtColor(cv_ptr->image,imgGray,CV_BGR2GRAY);
//Gray to contour
//GaussianBlur(imgGray,imgGray,Size(9,9),2,2);
//cv::Mat imgContour;
//Canny(imgGray,imgContour,10,250,5);
//imshow("canny",imgContour);

//RGB to HSV
cv::Mat imgHSV;
cvtColor(cv_ptr->image,imgHSV,CV_BGR2HSV);

//直方图均衡化
std::vector<Mat> hsvSplit;
split(imgHSV,hsvSplit);
equalizeHist(hsvSplit[2],hsvSplit[2]);
merge(hsvSplit,imgHSV);

//define a newWindow
namedWindow("Control",CV_WINDOW_AUTOSIZE);
int iLowH = 170;
int iHighH = 180;	

int iLowS = 70;
int iHighS = 255;

int iLowV = 50;
int iHighV = 255;


cvCreateTrackbar("LowH","Control",&iLowH,179);
cvCreateTrackbar("HighH","Control",&iHighH,179);

cvCreateTrackbar("LowS","Control",&iLowH,255);
cvCreateTrackbar("HighS","Control",&iHighS,255);

cvCreateTrackbar("LowV","Control",&iLowV,255);
cvCreateTrackbar("HighV","Control",&iHighV,255);

//二值化
cv::Mat imgThresholded;
cv::inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
//开操作
Mat element = getStructuringElement(MORPH_RECT,Size(5,5));
morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);

//闭操作
morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);
//imshow("Thresholded",imgThresholded);

//二值图片等高线
Mat imgContourThres;
Canny(imgThresholded,imgContourThres,10,250,5);
imshow("imgContour of Thresholded",imgContourThres);
//detect a circle
std::vector<Vec3f> circles;

//Mat imgThresholded_copy = imgThresholded.clone();
//HoughCircles(imgGray,circles,CV_HOUGH_GRADIENT,1.5,10,200,100,0,0)HoughCircles(imgThresholded,circles,CV_HOUGH_GRADIENT,1,10,150,70,0,0);
HoughCircles(imgContourThres,circles,CV_HOUGH_GRADIENT,1,10,100,20,10,0);
if (circles.size() != 0)
	{
	std::cout <<circles.size() <<" circles found out!"<< std::endl;
	}
count_circle = circles.size();	

float  x_pixel = 0.0;
float  y_pixel = 0.0;
for(size_t i =0;i<circles.size();i++)
{
	Point center(round(circles[i][0]),round(circles[i][1]));
	int radius = round(circles[i][2]);
	circle(cv_ptr->image,center,3,Scalar(0,255,0),-1,4,0);
	circle(cv_ptr->image,center,radius,Scalar(0,0,255),3,4,0);
	
	//计算中心点坐标
	 x_pixel += circles[i][0];
	 y_pixel += circles[i][1]; 
	
	
}
if (circles.size() != 0)
{

	x_pixel /= circles.size();
	y_pixel /= circles.size();
	std::cout<<"average x = "<<x_pixel<<std::endl; 
	std::cout<<"average y = "<<y_pixel<<std::endl;
	if (x_pixel < (X_MIDDLE-40))
		{
		//speed.linear.x = 0.1;
		speed.angular.z = 0.5;
		//speed.linear.y = 0.1;
		std::cout << "object found out in the left"<<std::endl;
		}
	else if (x_pixel > (X_MIDDLE+40))
		{
		//speed.linear.x = -0.1;
		//speed.linear.y = -0.1;
		speed.angular.z = -0.5;
		std::cout << "object found out in the right"<<std::endl;
		}
	else
		{
		speed.linear.x = 0;
		speed.angular.z = 0;
		speed.linear.y = 0;
		std::cout << "object found out in the middle, all finished!" << std::endl;

		std_msgs::String msg;
		msg.data = "True";

		isDetect.publish(msg);
		}
	cmdVelPub.publish(speed); 
	Point center_average(round(int(x_pixel)),round(int(y_pixel)));
	circle(cv_ptr->image,center_average,5,Scalar(255,0,0),-1,4,0);
}

cv::imshow("initial image",cv_ptr->image);
//cv::imshow("image Gray",imgGray);
// Update GUI Window
//cv::imshow(OPENCV_WINDOW,imgHSV);
cv::waitKey(3);
 
// Output modified video stream
image_pub_.publish(cv_ptr->toImageMsg());
average_x = x_pixel;
average_y = y_pixel;

}

//depth图的回调函数
void depthCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_depth_ptr;
	try{
	cv_depth_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
	
	//from float to int 
	double m,n;
	cv::Mat depthImage;
	//grayImage = msg->data;
	
	depthImage = cv_depth_ptr->image ;
	
//	cvMinMaxLoc(cv_depth_ptr->image,&m,&n,NULL,NULL,NULL);
//	cvConvertScale(cv_depth_ptr->image,out8U,255.0/(n-m), 255.0*(-m)/(n-m));

	//show the image and data
	 
	//auto result = cv_depth_ptr->image.at<cv::Vec3b>(1,1); 
	//std::cout << depthImage.at<float>(1,1)  << std::endl; 
	cv::imshow("depth image",cv_depth_ptr->image);
	// output the value of the depth
	float depthCenter = 0;
	if (count_circle != 0)
	{
		for(int i = -2; i < 3 ; i++ )
			for(int j = -2;j < 3;j++)						
				depthCenter += depthImage.at<float>(int(average_x+i),int(average_y+j));
				
		depthCenter /= 25;		
		std::cout << "depth of the center = "<< depthCenter<< std::endl;
	}
	 
	//std::cout << msg->encoding  << " * "  << msg->step << std::endl;
	

	// std::cout<<out8U<<std::endl;
	}
	catch(cv_bridge::Exception& e){
	ROS_ERROR("cv_bridge 异常: %s",e.what());
	return;
	}

	//std::cout << "depth of the circle = "<< grayImage[average[0]][average[1]]<< std::end


}
};
 
int main(int argc, char** argv)
{
ros::init(argc, argv, "image_converter");
ImageConverter ic;
ros::spin();
return 0;
}
