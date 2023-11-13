# PyTorch to ONNX

[TorchScript-based ONNX exporter](https://pytorch.org/docs/stable/onnx_torchscript.html#types)

## Tracing vs Scripting

`torch.onnx.export()`は内部的に`torch.nnModule`ではなく`torch.jit.ScriptModule`を必要とする。引数のモデルが`ScriptModule`でない場合、`export()`は*tracing*を実行する。

- **Tracing**

`torch.jit.trace()`と等価の処理。トレーシングではループ処理やif文が展開され、固定値として扱われる。

- **Scripting**

引数のモデルが`ScriptModel`(=`torch.jit.trace()`処理後のモデル)の場合実行される。

## Avoiding Pitfalls

### `NumPy`や組み込み型は避ける

`torch.Tensor`以外の型(=`NumPy`やPythonの組み込み型)は`tracing時に、固定値に変換され想定外の挙動になりうるので基本的に使わないようにする。

### `Tensor.data`は避ける

`torch.Tensor.data`は正しくトレースされない可能性があるので、代わりに`torch.Tensor.detach()`を使う。

### In-place処理は避ける

トレーシング時には、`tensor.shape`返り値はそれぞれ同一のメモリ領域を共有するtensorとして扱われるため、in-place処理を行うと正しく処理されない可能性がある。

```python
batch_size, dim_size = tensor1.shape[:2]

dim_size += 2 # Bad
dim_size = dim_size + 2 # Good
```

## Limitations

### Types

- `torch.Tensor`に変換可能な数値テンソルもしくは、それらの`list`または`tuple`のみがモデルの入出力として使える。
- `dict`や`str`はtracingで許容されているが、
  - 1回のtracing処理では**固定値として置き換えられる**
  - `dict`の出力では**keyが削除されて展開される**(e.g. `{"foo": 1, "bar": 2}` -> `(1, 2)`)
- `list`や`tuple`を含む特定の処理は、scriptingでネストに対する処理に制限があるため、サポートされていない。

### Unsupported Tensor Indexing Patterns

- 読み込み時

  ```python
  # indexを表すテンソルに負の値が含まれる
  data[torch.tensor([[1, 2], [2, -3]]), torch.tensor([-2, 3])]
  # 回避策: 正のindexを使う
  ```

- 書き込み時
  
  ```python
  # テンソルのrand >= 2の場合に複数ランクのindexがしてされている
  data[torch.tensor([[1, 2], [2, 3]]), torch.tensor([2, 3])] = new_data
  # 回避策: 1次元のテンソルを使う or rank == 1の複数の連続したテンソルを使う

  # 複数のテンソルのindexが非連続
  data[torch.tensor([2, 3]), :, torch.tensor([1, 2])] = new_data
  # 回避策: indexが連続するように`data`テンソルを転置する

  # indexを表すテンソルに負の値が含まれる
  data[torch.tensor([1, -2]), torch.tensor([-2, 3])] = new_data
  # 回避策: 正のindexを使う

  # `new_data`に暗黙のbroadcastが必要
  data[torch.tensor([[0, 2], [1, 1]]), 1:3] = new_data
  # 回避策: `new_data`を明示的に拡張する
  # 例:
  #     data shape: [3, 4, 5]
  #     new_data shape: [5]
  #     expected new_data shape after broadcasting: [2, 2, 2, 5]
  ```
## Adding support for operators

モデルがサポート外のオペレータを含む場合、以下のようなエラーメッセージが表示される。

```python
RuntimeError: ONNX export failed: Couldn't export operator foo
```

このような場合には以下の対応策がある
1. それらのオペレータを使わないようにモデルを変更する
2. それらのオペレータを変換するシンボリック関数を作成する
3. PyTorchにコントリビュートする

### [Symbolic関数](https://pytorch.org/docs/stable/onnx_torchscript.html#onnx-exporter-internals)

PyTorch内で定義されたオペレータをONNXオペレータの構成に分解する関数。

### [Atenオペレータ](https://pytorch.org/docs/stable/onnx_torchscript.html#aten-operators)

## [FAQ](https://pytorch.org/docs/stable/onnx_torchscript.html#frequently-asked-questions)

## [Python API](https://pytorch.org/docs/stable/onnx_torchscript.html#module-torch.onnx)

```python
torch.onnx.export(
    model, 
    args, 
    f, 
    export_params=True, 
    verbose=False, 
    training=<TrainingMode.EVAL: 0>, 
    input_names=None, 
    output_names=None, 
    operator_export_type=<OperatorExportTypes.ONNX: 0>, 
    opset_version=None, 
    do_constant_folding=True, 
    dynamic_axes=None, 
    keep_initializers_as_inputs=None, 
    custom_opsets=None, 
    export_modules_as_functions=False, 
    autograd_inlining=True
)
```