# mylib

Re-implementation of standard-library.

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

### Member types

| Name                     | Description                                       |
|:-------------------------|:--------------------------------------------------|
| `allocator_type`         | `Alloc`                                           |
| `traits`                 | `std::allocator_traits<allcator_type>`            |
| `value_type`             | `T`                                               |
| `size_type`              | `std::size_t`                                     |
| `pointer`                | `typename traits::pointer` (aka `T*`)             |
| `const_pointer`          | `typename traits::const_pointer` (aka `const T*`) |
| `reference`              | `T &`                                             |
| `const_reference`        | `const T&`                                        |
| `itertor`                | `T *`                                             |
| `const_iterator`         | `const T *`                                       |
| `reverse_iterator`       | `std::reverse_iterator<iterator>`                 |
| `const_reverse_iterator` | `std::reverse_itertor<const_iteartor>`            |

### Member functions

- :heavy_check_mark: : Completed
- :white_check_mark: : TODO update
- :x: : Not implemented yet

#### Construct/Deconstruct
| Name            | Description         | Implementation     |
|:----------------|:--------------------|:------------------:|
| `(constructor)` | Constructor         | :heavy_check_mark: |
| `(destructor)`  | Destructor          | :heavy_check_mark: |
| `operator=`     | Assignment operator | :heavy_check_mark: |

#### Iterator

| Name      | Description                                                         | Implementation     |
|:----------|:--------------------------------------------------------------------|:------------------:|
| `begin`   | Returns the iterator that points to the first element               | :heavy_check_mark: |
| `end`     | Returns the iterator that points to the last element                | :heavy_check_mark: |
| `cbegin`  | Returns the const iterator that points to the first element         | :heavy_check_mark: |
| `cend`    | Returns the const iterator that points to the last element          | :heavy_check_mark: |
| `rbegin`  | Returns the reverse iterator that points to the first element       | :heavy_check_mark: |
| `rend`    | Returns the reverse iterator that points to the last element        | :heavy_check_mark: |
| `crbegin` | Returns the const reverse iterator that points to the first element | :heavy_check_mark: |
| `crend`   | Returns the const reveres iterator that points to the last element  | :heavy_check_mark: |

#### Size

| Name            | Description                                                                            | Implementation     |
|:----------------|:---------------------------------------------------------------------------------------|:------------------:|
| `size`          | Returns the number of elements                                                         | :heavy_check_mark: |
| `capacity`      | Returns the number of maximum elements that can be stored without re-allocating memory | :heavy_check_mark: |
| `empty`         | Whether container is empty                                                             | :heavy_check_mark: |
| `resize`        | Resize the number of elemets                                                           | :heavy_check_mark: |
| `reserve`       | Update capacity                                                                        | :heavy_check_mark: |
| `shrink_to_fit` | Shrink capacity to the size                                                            | :x:                |

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
| `push_back`    | Add an element to the last         | :heavy_check_mark: |
| `emplace_back` | Create object to the last directly | :white_check_mark: |
| `clear`        | Clear all elements                 | :heavy_check_mark: |

## `mylib::allocator`

```cpp
namespace mylib {
    template<class T>
    class allocator;
}
```

### Member types

| Name            | Description   |
|:----------------|:--------------|
| `value_type`    | `T`           |
| `size_type`     | `std::size_t` |
| `pointer`       | `T*`          |
| `const_pointer` | `const T*`    |

### Member functions
| Name         | Description       | Implementation     |
|:-------------|:------------------|:------------------:|
| `allocate`   | Allocate memory   | :heavy_check_mark: |
| `deallocate` | Deallocate memory | :heavy_check_mark: |
