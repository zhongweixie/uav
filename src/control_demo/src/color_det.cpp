
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <control_demo/common.h>
#include <control_demo/color_det.h>
using namespace std;

class COLOR_DETECT
{
public:
  COLOR_DETECT()
  {
    COLOR_RANGE color_range("red", 20, 100, 100, 30, 255, 255);
    Color_range_vec.push_back(color_range);
    COLOR_RANGE color_range1("red", 20, 100, 100, 30, 255, 255);
    Color_range_vec.push_back(color_range1);
  }

  std::vector<DETECT_RESULT> detect(cv::Mat &img, cv::Mat &imgout, string &req_object)
  {

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    for (size_t i = 0; i < Color_range_vec.size(); i++)
    {
      if (req_object == "all" || req_object == Color_range_vec[i].Name)
      {
        cv::inRange(hsv, Color_range_vec[i].Lower, Color_range_vec[i].Upper, mask);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Rect> rects;
        for (size_t i = 0; i < contours.size(); i++)
        {
          cv::Rect rect = cv::boundingRect(contours[i]);
          rects.push_back(rect);
        }
      }
    }
  }
  std::vector<COLOR_RANGE> Color_range_vec;
};
