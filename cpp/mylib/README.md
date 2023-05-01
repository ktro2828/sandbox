# mylib

## `mylib::vector`

```cpp
namespace mylib {
    template<typename T, typename Alloc = std::allocator<T>>
    class vector;

    // Iterator
    template<typename T, typename Alloc = std::allocator<T>>
    class vector<T, Alloc>::iterator;
}
```

### Member functions

#### Construct/Deconstruct
| Name            | Description         | Implementation     |
|:----------------|:--------------------|:------------------:|
| `(constructor)` | Constructor         | :heavy_check_mark: |
| `(destructor)`  | Destructor          | :heavy_check_mark: |
| `operator=`     | Assignment operator | :heavy_check_mark: |

#### Iterator

| Name     | Description                                                   | Implementation     |
|:---------|:--------------------------------------------------------------|:------------------:|
| `begin`  | Returns the iterator that points to the first element         | :heavy_check_mark: |
| `end`    | Returns the iterator that points to the last element          | :heavy_check_mark: |
| `cbegin` | Returns the const iterator that points to the first element   | :heavy_check_mark: |
| `cend`   | Returns the const iterator that points to the last element    | :heavy_check_mark: |
| `rbegin` | Returns the reverse iterator that points to the first element | :heavy_check_mark: |
| `rend`   | Returns the reverse iterator that points to the last element  | :heavy_check_mark: |

#### Size

| Name       | Description                                                                            | Implementation     |
|:-----------|:---------------------------------------------------------------------------------------|:------------------:|
| `size`     | Returns the number of elements                                                         | :heavy_check_mark: |
| `resize`   | Resize the number of elemets                                                           | []                 |
| `capacity` | Returns the number of maximum elements that can be stored without re-allocating memory | :heavy_check_mark: |
| `empty`    | Whether container is empty                                                             | :heavy_check_mark: |
| `reserve`  | Update capacity                                                                        | []                 |

#### Access to the element

| Name         | Description                                            | Implementation     |
|:-------------|:-------------------------------------------------------|:------------------:|
| `operator[]` | Access to the element                                  | :heavy_check_mark: |
| `at`         | Access to the element                                  | :heavy_check_mark: |
| `front`      | Returns the reference that points to the first element | :heavy_check_mark: |
| `back`       | Returns the reference that points to the last element  | :heavy_check_mark: |
| `data`       | Returns the reference that points to the first element | :heavy_check_mark: |

#### Update container

| Name           | Description                        | Implementation     |
|:---------------|:-----------------------------------|:------------------:|
| `push_back`    | Add an element to the last         | []                 |
| `emplace_back` | Create object to the last directly | []                 |
| `clear`        | Clear all elements                 | :heavy_check_mark: |

## `mylib::allocator`

```cpp
namespace mylib {
    template<class T>
    class allocator;
}
```

### Member functions
| Name         | Description |
| :----------- | :---------- |
| `allocate`   | WIP         |
| `deallocate` | WIP         |
