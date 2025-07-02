# Custom C++ and CUDA Operators

[チュートリアルページ](https://docs.pytorch.org/tutorials/advanced/cpp_custom_ops.html)

## Setting up the Build System

[`torch.utils.cpp_extension`](https://pytorch.org/docs/stable/cpp_extension.html)を使うことでカスタムC++/CUDAコードをコンパイルできる。

```python
from setuptools import setup, Extension
from torch.utils import cpp_extension

setup(
    name="extension_cpp"
    ext_modules=[
        cpp_extension.CppExtension(
            "extension_cpp",
            ["muladd.cpp"],
            # define Py_LIMITED_API with min version 3.9 to expose only the stable
            # limited API subset from Python.h
            extra_compile_args={"cxx": ["-DPy_LIMITED_API=0x03090000"]},
            py_limited_api=True
        )
    ],
    cmdclass={"build_ext": cpp_extension.BuildExtension},
    options={"bdist_wheel": {"py_limited_api": "cp39"}}  # 3.9 is minimum supported Python version
)
```

CUDAコードをコンパイルする場合には、[`torch.utils.cpp_extension.CUDAExtension`](https://docs.pytorch.org/docs/stable/cpp_extension.html#torch.utils.cpp_extension.CUDAExtension)を使用する。使い方については、[extension-cpp](https://github.com/pytorch/extension-cpp)を参照。

## Define the custom op and adding backend implementations

例として、`mymuladd_cpu(...)`というカスタムオペレーターを実装する:

```cpp
at::Tensor mymuladd_cpu(at::Tensor a, const at::Tensor& b, double c) {
  TORCH_CHECK(a.sizes() == b.sizes());
  TORCH_CHECK(a.dtype() == at::kFloat);
  TORCH_CHECK(b.dtype() == at::kFloat);
  TORCH_INTERNAL_ASSERT(a.device().type() == at::DeviceType::CPU);
  TORCH_INTERNAL_ASSERT(b.device().type() == at::DeviceType::CPU);
  at::Tensor a_contig = a.contiguous();
  at::Tensor b_contig = b.contiguous();
  at::Tensor result = torch::empty(a_contig.sizes(), a_contig.options());
  const float* a_ptr = a_contig.data_ptr<float>();
  const float* b_ptr = b_contig.data_ptr<float>();
  float* result_ptr = result.data_ptr<float>();
  for (int64_t i = 0; i < result.numel(); i++) {
    result_ptr[i] = a_ptr[i] * b_ptr[i] + c;
  }
  return result;
}
```

カスタムのC++/CUDAコードをPyTorchのPythonフロントエンドで使用するには、オペレーターを`TORCH_LIBRARY` APIを使って登録する必要がある。
これによってオペレーターのPythonバインディングが自動的に生成される。

オペレーターの登録には2つのステップが必要:

- **オペレーター定義(`TORCH_LIBRARY`)**: これによりPyTorchがオペレーターを認識する。
- **バックエンド実装の登録(`TORCH_LIBRARY_IMPL`)**: これにより様々なバックエンド(CPUやCUDAなど)における実装がオペレーターに付与される。

## Defining an operator

以下のステップでオペレーター定義を行う:

1. オペレーターの**ネームスペースを指定する**、このときプロジェクト名を指定するのが推奨。(例: `extension_cpp`)
2. オペレーターの**入出力型のスキーマを指定する**。
   1. サポートされている型については、[The Custom Operators Manual](https://pytorch.org/docs/main/notes/custom_operators.html)を参照。
   2. 入力がミュータブルな場合は、[Creating mutable operators](https://docs.pytorch.org/tutorials/advanced/cpp_custom_ops.html#creating-mutable-operators)を参照してスキーマを指定する

```cpp
TORCH_LIBRARY(extension_cpp, m) {
    // Note that "float" in the schema corresponds to the C++ double type and the Python float type.
    m.def("mymuladd(Tensor a, Tensor b, float c) -> Tensor");
}
```

これによってオペレーターは、`torch.ops.extension_cpp.mymuladd`から使用可能になる。

## Registering backend implementations for an operator

`TORCH_LIBRARY_IMPL`を使うことでオペレーターのバックエンド実装を登録する。

```cpp
TORCH_LIBRARY_IMPL(extension_cpp, CPU, m) {
    m.impl("mymuladd", &mymuladd_cpu);
}
```

`mymuladd`にCUDA実装がある場合は、別途`TORCH_LIBRARY_IMPL`で登録する:

```cpp
__global__ void muladd_kernel(int numel, const float* a, const float* b, float c, float* result) {
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < numel) result[idx] = a[idx] * b[idx] + c;
}

at::Tensor mymuladd_cuda(const at::Tensor& a, const at::Tensor& b, double c) {
  TORCH_CHECK(a.sizes() == b.sizes());
  TORCH_CHECK(a.dtype() == at::kFloat);
  TORCH_CHECK(b.dtype() == at::kFloat);
  TORCH_INTERNAL_ASSERT(a.device().type() == at::DeviceType::CUDA);
  TORCH_INTERNAL_ASSERT(b.device().type() == at::DeviceType::CUDA);
  at::Tensor a_contig = a.contiguous();
  at::Tensor b_contig = b.contiguous();
  at::Tensor result = torch::empty(a_contig.sizes(), a_contig.options());
  const float* a_ptr = a_contig.data_ptr<float>();
  const float* b_ptr = b_contig.data_ptr<float>();
  float* result_ptr = result.data_ptr<float>();

  int numel = a_contig.numel();
  muladd_kernel<<<(numel+255)/256, 256>>>(numel, a_ptr, b_ptr, c, result_ptr);
  return result;
}

// ...
// CPU backend registration
// ...

TORCH_LIBRARY_IMPL(extension_cpp, CUDA, m) {
  m.impl("mymuladd", &mymuladd_cuda);
}
```

## Adding training(autograd) support for an operator

`torch.library.register_autograd`でオペレーターの学習時の挙動を追加でき、Pythonの`torch.autograd.Function`やC++の`torch::autograd::Function`を直接使うよりも簡単。

```python
def _backward(ctx, grad):
    a, b = ctx.saved_tensors
    grad_a, grad_b = None, None
    if ctx.needs_input_grad[0]:
        grad_a = grad * b
    if ctx.needs_input_grad[1]:
        grad_b = grad * a
    return grad_a, grad_b, None

def _setup_context(ctx, inputs, output):
    a, b, c = inputs
    saved_a, saved_b = None, None
    if ctx.needs_input_grad[0]:
        saved_b = b
    if ctx.needs_input_grad[1]:
        saved_a = a
    ctx.save_for_backward(saved_a, saved_b)

# This code adds training support for the operator. You must provide us
# the backward formula for the operator and a `setup_context` function
# to save values to be used in the backward.
torch.library.register_autograd(
    "extension_cpp::mymuladd", _backward, setup_context=_setup_context)
```

## Creating mutable operators

カスタムオペレーターの入力がミュータブルな場合は、スキーマ定義の際に`Tensor(a!)`を指定する。ミュータブルな入力が複数ある場合は`Tensor(a!), Tensor(b!), ...`のように異なる名前を使う。

```c++
// An example of an operator that mutates one of its inputs.
void myadd_out_cpu(const at::Tensor& a, const at::Tensor& b, at::Tensor& out) {
  TORCH_CHECK(a.sizes() == b.sizes());
  TORCH_CHECK(b.sizes() == out.sizes());
  TORCH_CHECK(a.dtype() == at::kFloat);
  TORCH_CHECK(b.dtype() == at::kFloat);
  TORCH_CHECK(out.dtype() == at::kFloat);
  TORCH_CHECK(out.is_contiguous());
  TORCH_INTERNAL_ASSERT(a.device().type() == at::DeviceType::CPU);
  TORCH_INTERNAL_ASSERT(b.device().type() == at::DeviceType::CPU);
  TORCH_INTERNAL_ASSERT(out.device().type() == at::DeviceType::CPU);
  at::Tensor a_contig = a.contiguous();
  at::Tensor b_contig = b.contiguous();
  const float* a_ptr = a_contig.data_ptr<float>();
  const float* b_ptr = b_contig.data_ptr<float>();
  float* result_ptr = out.data_ptr<float>();
  for (int64_t i = 0; i < out.numel(); i++) {
    result_ptr[i] = a_ptr[i] + b_ptr[i];
  }
}

TORCH_LIBRARY(extension_cpp, m) {
  m.def("myadd_out(Tensor a, Tensor b, Tensor(a!) out) -> ()");
}

TORCH_LIBRARY_IMPL(extension_cpp, CPU, m) {
  m.impl("myadd_out", &myadd_out_cpu);
}
```
