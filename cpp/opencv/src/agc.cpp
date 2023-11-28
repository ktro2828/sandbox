#include "agc.hpp"

AdaptiveGammaCorrector::AdaptiveGammaCorrector(const float alpha, const float threshold)
: alpha_(alpha), threshold_(threshold)
{
}

cv::Mat AdaptiveGammaCorrector::correct(const cv::Mat & rgb) const
{
  cv::Mat yCrCb;
  cv::cvtColor(rgb, yCrCb, cv::COLOR_RGB2YCrCb);

  const float mean_y = cv::mean(yCrCb)[0];
  constexpr float mean_exp = 112.f;
  const auto t = (mean_y - mean_exp) / mean_exp;

  cv::Mat dst;
  std::vector<cv::Mat> yCrCb_channels;
  if (t < -threshold_) {
    cv::split(yCrCb, yCrCb_channels);
    yCrCb_channels[0] = correct_dimmed(yCrCb_channels[0]);
    cv::merge(yCrCb_channels, dst);
    cv::cvtColor(dst, dst, cv::COLOR_YCrCb2RGB);
  } else if (threshold_ < t) {
    cv::split(yCrCb, yCrCb_channels);
    yCrCb_channels[0] = correct_bright(yCrCb_channels[0]);
    cv::merge(yCrCb_channels, dst);
    cv::cvtColor(dst, dst, cv::COLOR_YCrCb2RGB);
  } else {
    rgb.copyTo(dst);
  }

  return dst;
}

cv::Mat AdaptiveGammaCorrector::correct_bright(const cv::Mat & y_img) const
{
  cv::Mat inverse_y_img = y_img.clone();
  inverse_y_img = 255 - y_img;
  cv::Mat ret = gamma_correction(inverse_y_img, alpha_, false);
  return 255 - ret;
}

cv::Mat AdaptiveGammaCorrector::correct_dimmed(const cv::Mat & y_img) const
{
  return gamma_correction(y_img, 1.f - alpha_, true);
}

cv::Mat AdaptiveGammaCorrector::gamma_correction(
  const cv::Mat & y_img, const float gamma, const bool is_truncated) const
{
  // Calculate histogram
  cv::Mat hist_mat;
  int channels[] = {0};
  int histSize[] = {256};
  float range[] = {0, 256};
  const float * ranges[] = {range};
  cv::calcHist(&y_img, 1, channels, cv::Mat(), hist_mat, 1, histSize, ranges, true, false);

  std::vector<float> hist;
  for (int i = 0; i < hist_mat.rows; ++i) {
    hist.push_back(hist_mat.at<float>(i));
  }

  const auto sum_hist = std::accumulate(hist.cbegin(), hist.cend(), 0.0f);
  std::vector<float> prob_normalized(hist.size());
  for (size_t i = 0; i < hist.size(); ++i) {
    prob_normalized[i] = sum_hist == 0.0f ? 0.0f : hist[i] / (sum_hist);
  }

  const auto & [min_prob_ptr, max_prob_ptr] =
    std::minmax_element(prob_normalized.cbegin(), prob_normalized.cend());

  std::vector<float> tmp_prob(prob_normalized);
  for (auto & p : tmp_prob) {
    p = (p - *min_prob_ptr) / (*max_prob_ptr - *min_prob_ptr);
  }

  for (auto & prob : tmp_prob) {
    if (prob > 0.0f) {
      prob = *max_prob_ptr * std::pow(prob, gamma);
    } else if (prob < 0.0f) {
      prob = *max_prob_ptr * -std::pow(-prob, gamma);
    }
  }

  const float sum_prob = std::accumulate(tmp_prob.cbegin(), tmp_prob.cend(), 0.0f);
  std::vector<float> prob_normalized_wd(tmp_prob.size());
  for (size_t i = 0; i < tmp_prob.size(); ++i) {
    prob_normalized_wd[i] = sum_prob == 0.0f ? 0.0f : tmp_prob[i] / (sum_prob);
  }

  std::vector<float> cdf_prob_normalized_wd(prob_normalized_wd.size());
  std::partial_sum(
    prob_normalized_wd.cbegin(), prob_normalized_wd.cend(), cdf_prob_normalized_wd.begin());

  std::transform(
    cdf_prob_normalized_wd.begin(), cdf_prob_normalized_wd.end(), cdf_prob_normalized_wd.begin(),
    [](const auto & v) { return std::clamp(v, 0.0f, 1.0f); });

  std::vector<float> inverse_cdf = get_inverse_cdf(cdf_prob_normalized_wd, is_truncated);

  cv::Mat ret_y = y_img.clone();
  ret_y.convertTo(ret_y, CV_32F);
  std::vector<int> unique_intensity = get_unique_intensity<int>(y_img);
  for (const auto & i : unique_intensity) {
    const auto & intensity = static_cast<float>(i);
    const auto & inv_cdf = inverse_cdf.at(i);
    const float new_value = std::round(255.f * std::pow(intensity / 255.f, inv_cdf));
    ret_y.setTo(new_value, y_img == intensity);
  }
  ret_y.convertTo(ret_y, CV_8U);
  return ret_y;
}