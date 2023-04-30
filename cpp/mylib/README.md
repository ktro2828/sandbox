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
| Name            | Description         | Implementation |
|:----------------|:--------------------|:--------------:|
| `(constructor)` | Constructor         | []             |
| `(destructor)`  | Destructor          | []             |
| `operator=`     | Assignment operator | []             |

#### Iterator

| Name    | Description                                           | Implementation |
|:--------|:------------------------------------------------------|:--------------:|
| `begin` | Returns the iterator that points to the first element | []             |
| `end`   | Returns the iterator that points to the last element  | []             |

#### Size

| Name       | Description                                                                            | Implementation |
|:-----------|:---------------------------------------------------------------------------------------|:--------------:|
| `size`     | Returns the number of elements                                                         | []             |
| `resize`   | Resize the number of elemets                                                           | []             |
| `capacity` | Returns the number of maximum elements that can be stored without re-allocating memory | []             |
| `empty`    | Whether container is empty                                                             | []             |
| `reserve`  | Update capacity                                                                        | []             |

#### Access to the element

| Name         | Description                                            | Implementation |
|:-------------|:-------------------------------------------------------|:--------------:|
| `operator[]` | Access to the element                                  | []             |
| `at`         | Access to the element                                  | []             |
| `data`       | Returns the reference that points to the first element | []             |

#### Update container

| Name           | Description                        | Implementation |
|:---------------|:-----------------------------------|:--------------:|
| `push_back`    | Add an element to the last         | []             |
| `emplace_back` | Create object to the last directly | []             |

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
