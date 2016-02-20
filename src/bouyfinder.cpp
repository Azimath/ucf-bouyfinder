#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <features.h>

//using namespace cv;

static const char* OPENCV_WINDOW = "Image window";
static const std::string CIRCLE_WINDOW = "Circle Window";

int iLowH = 0;
int iHighH = 179;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

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
        cv::namedWindow(CIRCLE_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat imageGrey;

        cvCreateTrackbar("LowR", OPENCV_WINDOW, &iLowH, 255);
        cvCreateTrackbar("HighR", OPENCV_WINDOW, &iHighH, 255);

        cvCreateTrackbar("LowG", OPENCV_WINDOW, &iLowS, 255);
        cvCreateTrackbar("HighG", OPENCV_WINDOW, &iHighS, 255);

        cvCreateTrackbar("LowB", OPENCV_WINDOW, &iLowV, 255);
        cvCreateTrackbar("HighB", OPENCV_WINDOW, &iHighV, 255);

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::cvtColor(cv_ptr->image, imageGrey, CV_BGR2GRAY);

        cv::GaussianBlur(imageGrey, imageGrey, cv::Size(9,9), 2, 2);
        cv::vector<cv::Vec3f> circles;
        cv::HoughCircles(imageGrey, circles, CV_HOUGH_GRADIENT,2,imageGrey.rows/4, 200, 100);

        cv::Mat maskedImage = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
        cv::Mat HSVImage;
        cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);

        int bestHSimilarity = 179;
        cv::Vec3f bestCircle;

        for(size_t i = 0; i < circles.size(); i++)
        {
            cv::Mat mask;
            mask = cv::Mat::zeros(imageGrey.rows, imageGrey.cols, CV_8U);

            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            cv::circle(mask, center, radius, cv::Scalar(255),-1,8,0);

            cv::Scalar meanColor;
            meanColor = mean(HSVImage, mask);

            if(iLowH < meanColor[0] && meanColor[0] < iHighH && iLowS < meanColor[1] && meanColor[1] < iHighS && iLowV < meanColor[2] && meanColor[2] < iHighV)
            {
                cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
                cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255),3,8,0);
                if(std::abs(meanColor[0]-(iLowH+iHighH)/2)<bestHSimilarity)
                {
                    bestHSimilarity = std::abs(meanColor[0]-(iLowH+iHighH)/2);
                    bestCircle = circles[i];
                }
            }
        }

        ROS_INFO("Found best circle with X, Y, Size, H: %f, %f, %f, %f", bestCircle[0], bestCircle[1], bestCircle[2], bestHSimilarity);
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        imshow(CIRCLE_WINDOW, maskedImage);
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
