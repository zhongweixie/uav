
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <control_demo/common.h>
#include <chrono>
using namespace std;

struct COLOR_RANGE
{
  std::string Name;
  cv::Scalar Lower;
  cv::Scalar Upper;
  COLOR_RANGE(string name, int lower1, int lower2, int lower3, int upper1, int upper2, int upper3) : Lower(lower1, lower2, lower3), Upper(upper1, upper2, upper3)
  {
    Name = name;
  }
};

class COLOR_DETECT
{
public:
  COLOR_DETECT()
  {
    COLOR_RANGE color_range("red", 153, 43, 46 ,180, 255,255);
    Color_range_vec.push_back(color_range);
    COLOR_RANGE color_range1("green", 100, 100,100, 100, 100, 100);
    Color_range_vec.push_back(color_range1);
  }

  std::vector<DETECT_RESULT> detect(cv::Mat &img, cv::Mat &imgout,string req_object)
  {
    auto start = std::chrono::system_clock::now();
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    std::vector<DETECT_RESULT> result_vec;
    for (size_t i = 0; i < Color_range_vec.size(); i++)
    {
      if(Color_range_vec[i].Name==req_object){
      cv::inRange(hsv, Color_range_vec[i].Lower, Color_range_vec[i].Upper, mask);
     // cv::imshow("mask", mask);
	
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      for (size_t j = 0; j < contours.size(); j++)
      {
      // cout << "H: " << (int)hsv.at<cv::Vec3b>( contours[j][0].y, contours[j][0].x)[0]  << '\t';
			//cout << "S: " << (int)hsv.at<cv::Vec3b>( contours[j][0].y, contours[j][0].x)[1]  << '\t';
		//	cout << "V: " <<(int) hsv.at<cv::Vec3b>( contours[j][0].y, contours[j][0].x)[2]<< endl;
        DETECT_RESULT detect_result;
        detect_result.Rect = cv::boundingRect(contours[j]);
        detect_result.name = Color_range_vec[i].Name;
        detect_result.Id = j;

        result_vec.push_back(detect_result);
      }
      }
    }
    imgout = img;
    for (size_t k = 0; k < result_vec.size(); k++)
    {
      cv::rectangle(imgout, result_vec[k].Rect, cv::Scalar(0x27, 0xC1, 0x36), 2);
      cv::putText(imgout, result_vec[k].name, cv::Point(result_vec[k].Rect.x, result_vec[k].Rect.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }
    auto end = std::chrono::system_clock::now();
    //std::cout << "process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    return result_vec;
  }
  std::vector<COLOR_RANGE> Color_range_vec;
};
