#ifndef OPENCV_TEST_DEBUG_HPP_
#define OPENCV_TEST_DEBUG_HPP_

#include <opencv2/opencv.hpp>

#include <iostream>

void print(const cv::Mat & img)
{
  std::cout << cv::format(img, cv::Formatter::FMT_NUMPY) << std::endl;
}

template <typename T>
void print(const std::vector<T> & data)
{
  std::cout << cv::format(data, cv::Formatter::FMT_NUMPY) << std::endl;
}

void print_dtype(const cv::Mat & img, const char * msg = nullptr)
{
  int dataType = img.type();

  // データ型に対応する文字列を取得
  std::string dataTypeString;
  switch (dataType) {
    case CV_8U:
      dataTypeString = "CV_8U(C1)";
      break;
    case CV_8UC2:
      dataTypeString = "CV_8UC2";
      break;
    case CV_8UC3:
      dataTypeString = "CV_8UC3";
      break;
    case CV_8S:
      dataTypeString = "CV_8S";
      break;
    case CV_16U:
      dataTypeString = "CV_16U(C1)";
      break;
    case CV_16UC2:
      dataTypeString = "CV_16UC2";
      break;
    case CV_16UC3:
      dataTypeString = "CV_16UC3";
      break;
    case CV_16S:
      dataTypeString = "CV_16S(C1)";
      break;
    case CV_16SC2:
      dataTypeString = "CV_16SC2";
      break;
    case CV_16SC3:
      dataTypeString = "CV_16SC3";
      break;
    case CV_32S:
      dataTypeString = "CV_32S(C1)";
      break;
    case CV_32SC2:
      dataTypeString = "CV_32SC2";
      break;
    case CV_32SC3:
      dataTypeString = "CV_32SC3";
      break;
    case CV_32F:
      dataTypeString = "CV_32F(C1)";
      break;
    case CV_32FC2:
      dataTypeString = "CV_32FC2";
      break;
    case CV_32FC3:
      dataTypeString = "CV_32FC3";
      break;
    case CV_64F:
      dataTypeString = "CV_64F(C1)";
      break;
    case CV_64FC2:
      dataTypeString = "CV_64FC2";
      break;
    case CV_64FC3:
      dataTypeString = "CV_64FC3";
      break;
    default:
      dataTypeString = "Unknown";
  }

  if (msg) {
    std::string str(msg);
    std::cout << str << " ";
  }

  std::cout << dataTypeString << std::endl;
}
#endif  // OPENCV_TEST_DEBUG_HPP_