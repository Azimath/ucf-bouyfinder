#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string EDGE_WINDOW = "Edge Window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {

    //Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw",1,
			       &ImageConverter::imageCb,this);
    image_pub_ = it_.advertise("/image_converter/output_video",1);

    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(EDGE_WINDOW);
    ROS_INFO("constructed");
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat gray;
    cv::Mat edges;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }
    //if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));
    
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(9,9), 2, 2);
    cv::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,2,gray.rows/4, 100, 100);
    for(size_t i = 0; i < circles.size(); i++)
      {
	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	int radius = cvRound(circles[i][2]);
	cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
	cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255),3,8,0);
      }
    cv::Canny(gray, edges, 100, 50);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow(EDGE_WINDOW, edges);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
