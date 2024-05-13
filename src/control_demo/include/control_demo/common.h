#ifndef _COMMON_H  
#define _COMMON_H  
#include <string>
#include <opencv2/core/core.hpp>
using namespace std;

struct DETECT_RESULT
{
  std::string Det_type;
  std::string Info;
  cv::Rect Rect;
  int Id;
  string name;
};

  
#endif 

