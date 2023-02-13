#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <cstdint>
#include <string>
#include <memory>

using namespace nvinfer1;
using namespace std;
using namespace cudawrapper;

static nvinfer1::Logger gLogger;

// Number of times we run inference to calcuate average time.
constexpr int ITERATIONS = 10;
// Maximum absolute tolerance for output tensor comparison against reference.
constexpr double ABS_EPSILON = 0.005;
// Maximum relative torelance for output tensor comparison against reference.
constexpr double REL_EPSILON = 0.05;
// Allow TensorRT to use up to 1GB for GPU memory for tactic selection.
constexpr size_t MAX_WORKSPACE_SIZE = 1ULL << 30;  // 1GB

ICudaEngine* createCudaEngine(const string & onnx_model_path, int batch_size)
{
  const auto explicit_batch = 1U << static_cast<std::uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  unique_ptr<nvinfer1::IBuilder, Destroy<nvinfer1::IBuilder>> builder{nvinfer1::createInferBuilder(gLogger)};
  unique_ptr<nvinfer1::INetworkDefinition, Destroy<nvinfer1::INetworkDefinition>> network{builder->createNetworkV2(explicit_batch)};
  unique_ptr<nvonnxparser::IParser, Destroy<nvonnxparser::IParser>> parser{nvonnxparser::createParser(*network, gLogger)};
  unique_ptr<nvinfer1::IBuilderConfig, Destory<nvinfer1::IBuilderConfig>> config{builder->createBuilderConfig()};

  if (!parser->parseFronFile(onnx_model_path.c_str(), static_cast<int>(ILogger::Severity::kINFO)))
    {
      cout << "ERROR: counld not parse input engine." << endl;
      return nullptr;
    }

  config->setMaxWorkspaceSize(MAX_WORKSPACE_SIZE);
  builder->setFp16Mode(builder->platformHasFastFp16());
  builder->setMaxBatchSize(batch_size);

  auto profile = builder->createOptimizationProfile();
  profile->setDimensions(network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4{1, 3, 256, 256});
  profile->setDimensions(network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kOPT, nvinfer1::Dims4{1, 3, 256, 256});
}
