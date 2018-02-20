#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

static const std::string OPENCV_WINDOW = "Image window";

Size boardSize = Size(7,5);
string dirPath = "/home/marc/Workspace/ROS/realsense_ws/src/image_creation/images/";
int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;
int imageCounter = 1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // vector of 2D features
  vector<Point2f> pointBuf;
    
  // compression flags
  vector<int> compression_params;

  // capture flag
  bool capturing;


public:
  ImageConverter()
    : it_(nh_)
  {
    
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("in", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("out", 1);

    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    capturing = false;

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat view = cv_ptr->image;

    auto keyPressed = (char)waitKey(1);

    if(capturing || keyPressed == 'c'){
            
      if(keyPressed == 'c')
        capturing = !capturing;

      bool found = findChessboardCorners(view, boardSize, pointBuf, chessBoardFlags);

      if(found){
                  Mat viewGray;
                  // Convert image to grey scale
                  cvtColor(view, viewGray, COLOR_BGR2GRAY);
                  // Use sub pixel interpolation to find more accurate corener estimates
                  cornerSubPix( viewGray, pointBuf, Size(11,11),
                              Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1 ));
                  // draw corners (Pointbuf) into image
                  drawChessboardCorners( viewGray, boardSize, Mat(pointBuf), found );
                  imshow(OPENCV_WINDOW, viewGray);

                  try {
                        string imageName(dirPath + "image_" + to_string(imageCounter)+".png");
                        imwrite(imageName, view, compression_params);
                        cout << imageName << " saved to disk" << std::endl;
                        imageCounter++;
                    }
                    catch (runtime_error& ex) {
                        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
                    }
      }
      else
        imshow(OPENCV_WINDOW, view);
    }

    else
      imshow(OPENCV_WINDOW, view);    



    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}