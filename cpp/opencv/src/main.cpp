#include "agc.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>

int main()
{
  cv::Mat img = cv::imread("./backlit.png");
  cv::imshow("img", img);
  cv::waitKey(0);

  cv::Mat rgb;
  cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);

  cv::Mat yCrCb;
  cv::cvtColor(rgb, yCrCb, cv::COLOR_RGB2YCrCb);
  std::cout << yCrCb.type() << std::endl;
  cv::imshow("yCrCb", yCrCb);
  cv::waitKey(0);

  AdaptiveGammaCorrector corrector(0.25f, 0.3f);
  std::cout << ">> Start correction" << std::endl;
  cv::Mat ret_rgb = corrector.correct(rgb);
  cv::Mat ret_bgr;
  cv::cvtColor(ret_rgb, ret_bgr, cv::COLOR_RGB2BGR);
  cv::imshow("ret", ret_bgr);
  cv::waitKey(0);
}