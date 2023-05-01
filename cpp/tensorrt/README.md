# TensorRT

TensorRTとは，NVIDIAによって開発された高速な機械学習推論用のライブラリである．
NVIDIA製のGPU上での機械学習モデルの推論の最適化・高速化を行う．

TensorRTは，TensorFlow, PyTorch, Caffeといった機械学習フレームワークで学習されたモデルの最適化と実行，量子化，subgraph execution，layer fusion等ができる．(参考: [畳み込みNNの高速化について](https://developer.smartnews.com/blog/2017/06/convolution-speed-up/))

see [ktro2828/trt-sandbox](https://github.com/ktro2828/trt-sandbox.git)

## サンプル

```cpp
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

constexpr int BATCH_SIZE = 1;
constexpr int INPUT_CHANNELS = 3;
constexpr int INPUT_HEIGHT = 224;
constexpr int INPUT_WIDTH = 224;
constexpr int OUTPUT_CHANNELS = 1000;

constexpr std::string ONNX_FILE = "YOUR_MODEL.onnx"

constexpr size_t MAX_WORKSPACE_SIZE = 1ULL << 30;  // 1GB

namespace nvinfer1
{
// nvinfer1::ILoggerを継承したLoggerクラス
// ILogger::log()をオーバーライドする
class Logger : public nvinfer1::ILogger
{
private:
    bool verbose_{false};

public:
    explicit Logger(bool verbose) : verbose_(verbose) {}

    virtual void log(Severity severity, const char* msg) noexcept override
    {
        if (verbose_ || (serverity != Serverity::kINFO) && (serverity != Serverity::kVERBOSE))
        {
            std::cerr << severity << ": " << msg << std::endl;
        }
    }
};

// unique_ptrを使う際の独自の deleter
class Deleter
{
    template <typename T>
    void operator()(T* obj) const
    {
        if (obj)
        {
            // obj->destroy()はdeprecated
            delete obj;
        }
    }
};
}

static Logger logger(true);

template<typename T>
using unique_ptr = std::unique_ptr<T, nvinfer1::Deleter>


void setupConfig(nvinfer1::IBuilderConfig *config, const nvinfer1::IBuilder &builder)
{
    #if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
        config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, MAX_WORKSPACE_SIZE);
    #else
        config->setMaxWorkspaceSize(MAX_WORKSPACE_SIZE);
    #endif

    // set profile
    auto profile = builder->createOptimizationProfile();
    profile->setDimensions(network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMIN, nvinfer1::Dims4{BATCH_SIZE, INPUT_CHANNELS, INPUT_HEIGHT, INPUT_WIDTH});
    profile->setDimensions(network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kOPT, nvinfer1::Dims4{BATCH_SIZE, INPUT_CHANNELS, INPUT_HEIGHT, INPUT_WIDTH});
    profile->setDimensions(network->getInput(0)->getName(), nvinfer1::OptProfileSelector::kMAX, nvinfer1::Dims4{BATCH_SIZE, INPUT_CHANNELS, INPUT_HEIGHT, INPUT_WIDTH});
    config->addOptimizationProfile(profile);
}

void setupContext(nvinfer1::IExecutionContext * context, nvinfer1::ICudaEngine * engine)
{
    nvinfer1::Dims dims_i{engine->getBindingDimensions(0)};
    nvinfer1::Dims4 inputDims{BATCH_SIZE, dims_i.d[1], dims_i.d[2], dims_i.d[3]};
    context->setBindingDimensions(0, inputDims);
}

void allocateOnCuda(void** bindings, nvinfer1::ICudaEngine* engine)
{
    for (int i = 0; i < engine->getNbBindings(); ++i)
    {
        nvinfer1::Dims dims{engine->getBindingDimensions(i)};
        size_t size = std::accumulate(dims.d+1, dims.d + dims.nbDims, BATCH_SIZE, std::multiplies<size_t>());
        // Memory allocation
        cudaMalloc(&bindings[i], BATCH_SIZE * size * sizeof(float));
    }
}

static int getBindingInputIndex(nvinfer1::IExecutionContext* contex)
{
    return !context->getEngine().bindingsIsInput(0);
}

void inference(nvinfer1::IExecutionContext *context, CudaStream_t stream, void** bindings, const std::vector<float> &input, const std::vector<float> &output)
{
    cudaEvent start, end;
    cudaEventRecord(start, stream); // event start
    int inputId = getBindingInputIndex(context);
    // Copy data from Host to Device
    cudaMemcpyAsync(bindings[inputId], input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, stream);
    context->enqueueV2(bindings, stream, nullptr);
    // Copy data from Device to Host
    cudaMemcpyAsync(output.data(), bindings[1 - inputId], output.size() * sizeof(float), cudaMemcpyDeviceToHost, stream);
    cudaEventRecord(end, stream); // event end
}

int main()
{

    unique_ptr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger)};
    unique_ptr<nvinfer1::IBuilder> builder{nvinfer1::createInferBuilder(logger)};
    builder->setFp16Mode(builder->platformHasFastFp16);
    builder->setMaxBatchSize(BATCH_SIZE);

    const auto flag = 1U << static_cast<std::uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    unique_ptr<nvinfer1::INetworkDefinition> network{builder->createNetworkV2(flag)};

    // Create parser and parse onnx model
    unique_ptr<nvonnxparser::IParser> parser{nvonnxparser::createParser(*network, logger)};
    if (!parser->parserFromFile(ONNX_FILE.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kERROR));)
    {
        std::cout << "ERROR: could not parse input engine." << std::endl;
        return 0;
    }

    // Create config
    unique_ptr<nvinfer1::IBuilderConfig> config{builder->createBuilderConfig()};
    setupConfig(config.get(), builder.get());

    // Build engine
    unique_ptr<nvinfer1::IHostMemory> plan{builder->buildSerializedNetwork(*network, *config)};
    unique_ptr<nvinfer1::ICudaEngine> engine{runtime->deserializeCudaEngine(plan->data(), plan->size())};

    void* bindings[2]{0};
    allocateOnCuda(bindings.get(), engine.get());

    // Create context
    unique_ptr<nvinfer1::IExecutionContext> context{engine->createExecutionContext()};
    setupContext(context.get(), engine.get());

    // Inference
    std::vector<float> input(BATCH_SIZE*INPUT_CHANNELS*INPUT_HEIGHT*INPUT_WIDTH);
    std::vector<float> output(BATCH_SIZE*OUTPUT_CHANNELS);

    // CUDA
    CudaStream stream;
    inference(context.get(), stream, bindings, input, output);

    // Free all allocated memories
    for (*void ptr: bindings)
    {
        cudaFree(ptr);
    }
}
```

## CUDA API
### `cudaMalloc`/`cudaFree`
デバイスメモリにデータを格納する領域を確保・解放する．

```cpp
cudaError_t cudaMalloc(
    void** devPtr,
    size_t size
)

cudaError_t cudaFree(void* devPtr)

// e.g.) データ長1024のint配列を確保・解放
size_t N = 1024;
size_t size = N * sizeof(int);
int *d_a = 0;
cudaMalloc((void**)&d_a, size);
cudaFree(d_a);
```
- `devPtr`: 確保するデバイスメモリ上のアドレスへのポインタ
- `size`：確保する領域のデータ長(バイト)

### `cudaMemcpy`

`cudaMalloc`で確保したデバイスメモリ上の領域へホストメモリからデータを転送，またはその逆の操作を行う．

```cpp
cudaError_t cudaMemcpy(
    void* dst,
    const void* src,
    size_t count,
    cudaMemcpyKind kind,
)
```
- `dst`: コピー先メモリのポインタ．
- `src`: コピー元メモリのポインタ．
- `count`: コピーされるデータ長(バイト)．
- `kind`: メモリコピー方法．
  - `cudaMemcpyHostToHost`: ホストメモリ(CPU)からホストメモリへのコピー
  - `cudaMemcpyHostToDevice`: ホストメモリからデバイスメモリ(GPU)へのコピー
  - `cudaMemcpyDeviceToHost`: デバイスメモリからホストメモリへのコピー
  - `cudaMemcpyDeviceToDevice`: デバイスメモリからデバイスメモリへのコピー

`cudaMemcpy`は同期関数であるため，コピーが完全に終わるまで次の処理に移行できない．
そこで非同期関数として，`cudaMemcpyAsync`がある．

```cpp
cudaError_t cudaMemcpyAsync(
    void* dst,
    const void* src,
    size_t count,
    cudaMemcpyKind kind,
    cudaStream_t stream = 0,
)
```

## References
- [NVIDIA Deep Learning TensorRT Documentation](https://docs.nvidia.com/deeplearning/tensorrt/index.html)
  - [Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html)
- [How to Speed up Deep Learning Inference Using TensorRT~NVIDIA DEVELOPER Technical Blog~](https://developer.nvidia.com/blog/speed-up-inference-tensorrt/)
