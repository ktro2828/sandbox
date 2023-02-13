#include <iostream>
#include <NvOnnxParser.h>

#include "network.hpp"
#include "ioHelper.hpp"

Network::Network(const std::string &path, bool verbose)
{
  Logger logger(verbose);
  runtime_ = std::unique_ptr<nvinfer1::IRuntime, Delter<nvinfer1::IRuntime>>(nvinfer1::createInferRuntime(logger));
  loadFromFile(path);
  if (!prepare()) {
    std::cout << "ERROR: Fail to prepare engine." << std::endl;
    return;
  }
}

Network::~Network()
{}

Network::Network(const std::string &onnx_file_path, const std::string &precision, const int max_batch_size, bool verbose, size_t workspace_size)
{
  Logger logger(verbose);
  runtime_ = std::unique_ptr<nvinfer1::IRuntime, Delter<nvinfer1::IRuntime>>(nvinfer1::createInferRuntime(logger));
  if (!runtime_)
    {
      std::cout << "ERROR: Fail to create runtime" << std::endl;
      return;
    }
  bool fp16 = precision.compare("FP16") == 0;
  bool int8 = precision.compare("INT8") == 0;

  // Create builder
  auto builder = std::unique_ptr<nvinfer1::IBuilder, Deleter<nvinfer1::IBuilder>>(nvinfer1::createInferBuilder(logger));
  if (!builder)
    {
      std::cout << "ERROR: Fail to create builder" << std::endl;
      return;
    }
  auto config = std::unique_ptr<nvinfer1::IBuilderConfig, Deleter<nvinfer1::IBuilderConfig>>(nvinfer1::createBuilderConfig());
  if (!config)
    {
      std::cout << "ERROR: Fail to create builder config" << std::endl;
      return;
    }
  // Allow use of FP16 layers when running in INT8
  if (fp16 || int8)
    {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
  // If tensorRT > 8.4.0, use config->setMemoryPoolLimit()
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size);
#else
  config->setMaxWorkspaceSize(workspace_size);
#endif

  // Parse ONNX FCN
  std::cout << "Building " << precision << " core model..." << std::endl;
  const auto flat = 1U << static_cast<std::uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(builder->createNetworkV2(flag));
  if (!network)
    {
      std::cout << "ERROR: Fail to create network" << std::endl;
      return;
    }
  auto parser = std::unique_ptr<nvonnxparser::IParser>(nvonnxparser::createParser(*network, logger));
  if (!parser)
    {
      std::cout << "ERROR: Fail to create parser" << std::endl;
      return;
    }

  parser->parseFromFile(onnx_file_path.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));
  const auto input = network->getInput(0);
  const auto num
				 
}

