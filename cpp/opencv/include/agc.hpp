#ifndef OPENCV_TEST_AGC_HPP_
#define OPENCV_TEST_AGC_HPP_

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <numeric>

class AdaptiveGammaCorrector
{
public:
  explicit AdaptiveGammaCorrector(const float alpha, const float threshold);

  cv::Mat correct(const cv::Mat & img) const;

private:
  cv::Mat correct_bright(const cv::Mat & y_img) const;

  cv::Mat correct_dimmed(const cv::Mat & y_img) const;

  cv::Mat gamma_correction(const cv::Mat & y_img, const float gamma, const bool is_truncated) const;

  template <typename T>
  std::vector<T> get_unique_intensity(const cv::Mat & src) const
  {
    cv::Mat intensity = src.reshape(1, 1);
    intensity.convertTo(intensity, CV_32F);
    std::set<float> unique_values(intensity.begin<float>(), intensity.end<float>());
    std::vector<T> out(unique_values.begin(), unique_values.end());
    std::sort(out.begin(), out.end());
    return out;
  }

  template <typename T>
  std::vector<T> get_inverse_cdf(const std::vector<T> & cdf, const bool is_truncated) const
  {
    std::vector<T> out(cdf.size());
    for (size_t i = 0; i < cdf.size(); ++i) {
      out[i] = is_truncated ? std::max(0.5f, 1.0f - cdf.at(i)) : 1.0f - cdf.at(i);
    }
    return out;
  }

  float alpha_;
  float threshold_;
};  // class AdaptiveGammaCorrector

#endif  // OPENCV_TEST_AGC_HPP_