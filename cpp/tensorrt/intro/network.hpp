#ifndef __NETROWK__HPP__
#define __NETROWK__HPP__

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <cuda_runtim_api.h>

#include <memory>
#include <string>
#include <vector>

#include "ioHelper.hpp"

class Network
{
private:
  std::unique_ptr<nvinfer1::IRuntime, Deleter<nvinfer1::IRuntime>> runtime_ = nullptr;
  std::unique_ptr<nvinfer1::IHostMemory, Deleter<nvinfer1::IHostMemory>> plan_ = nullptr;
  std::unique_ptr<nvinfer1::ICudaEngine, Deleter<nvinfer1::ICudaEngine>> engine_ = nullptr;
  std::unique_ptr<nvinfer1::IExecutionContext, Deleter<nvinfer1::IExecutionContext>> context_ = nullptr;
  std::unique_ptr<float[], Deleter<float[]>> input_dim_ = nullptr;
  std::unique_ptr<float[], Deleter<float[]>> output_dim_ = nullptr;

  void loadFromFile(const std::string &path);
  bool prepare();
  std::vector<float> preprocess(const cv::Mat &img, const int channel, const int height, const int width) const;
  void inference(std::vector<void*> &buffers, const int batch_size);

public:
  // Create engine from engine file
  explicit Network(const std::string &engine_path, bool verbose = false);
  // Create engin from serialized onnx model
  Network(const std::string &onnx_file_path, const std::string &precision, const int max_batch_size, bool verbose = false, size_t workspace_size = (1ULL << 30));

  ~Network();

  void save(const std::string &path) const;
  bool detect(const cv::Mat &img);

  std::vector<int> getInputDims() const;
  int getMaxBatchSize() const;
  int getInputSize() const;
  
}; // class Network

#endif // __NETROWK__HPP__
