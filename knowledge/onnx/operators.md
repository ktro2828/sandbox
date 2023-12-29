# ONNX Operators

## Supported operators

[The list of supported ONNX operators](https://github.com/onnx/onnx/blob/main/docs/Operators.md)

### NonZero

Returns the indices of the elements that non-zero(in row-major order - by dimension), similar to [`np.nonzero`](https://numpy.org/doc/stable/reference/generated/numpy.nonzero.html).

### ScatterND

The `output` is calculated as follows:

```python
output = np.copy(data)
update_indices = indices.shape[:-1]
for idx in np.ndindex(update_indices):
    output[indices[idx]] = updates[idx]
```

Example:

```python
data    = [1, 2, 3, 4, 5, 6, 7, 8]
indices = [[4], [3], [1], [7]]
updates = [9, 10, 11, 12]
output  = [1, 11, 3, 10, 9, 6, 7, 12]
```

## [Adding support for operators](https://glaringlee.github.io/onnx.html#adding-support-for-operators)

### ATen operators

追加したいonnxオペレータがATenオペレータの場合(=`torch/csrc/autograd/generated/VariableType.h`に定義されている)、`torch/onnx/symbolic_opset<version>.py`に追加する。
追加する際は、以下に従う。

## Non-ATen operators