#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <control_demo/common.h>
#include <chrono>
using namespace std;



class CIRCLE_DETECT
{
public:
  CIRCLE_DETECT()
  {

  }

  std::vector<DETECT_RESULT> detect(cv::Mat &img, cv::Mat &imgout,string req_object)
  {
    auto start = std::chrono::system_clock::now();


    cv::Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 85, 0, 0);
    std::vector<DETECT_RESULT> result_vec;
    DETECT_RESULT detect_result;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(img, center, radius,cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        cv::Rect rect(center.x - radius, center.y - radius, radius * 2, radius * 2);
        rectangle(img, rect, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        detect_result.Rect = rect;
        detect_result.name = req_object;
        detect_result.Id = i;
        result_vec.push_back(detect_result);

    }
  
 

    imgout = img;

    auto end = std::chrono::system_clock::now();
    //std::cout << "process time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
    return result_vec;
  }

};
