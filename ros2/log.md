# Log

## `RCLCPP_<LEVEL>_STREAM`?

- `RCLCPP_<LEVEL>` : `printf`形式で記述
- `RCLCPP_<LEVEL>_STREAM` : `std::ostream <<`形式で記述

```cpp
// printf style
RCLCPP_DEBUG(node->get_logger(), "My log message %d", 4);

// C++ stream style
RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4);
```

## 1回だけ出力する

```c++
// printf style
RCLCPP_INFO_ONCE(node->get_logger(), "My log message once %d", 4);

// C++ stream style
RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "My log message once " << 4);
```

## 2回目以降は出力する(1回目は出力しない)

```c++
// printf style
RCLCPP_WARN_SKIPFIRST(node->get_logger(), "My warning message after the second time %d", 4);

// C++ stream style
RCLCPP_WARN_STREAM_SKIPFIRST(node->get_logger(), "My warning message after the second time " << 4);
```

## `N[ms]`に1回だけ出力

```c++
// 1000[回/ms] = 1[回/s]

// printf style
RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My error message once per second %d", 4);

// C++ stream style
RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My error message once per second" << 4);
```

## 2回目以降は`N[ms]`に1回だけ出力

```c++
// printf style
RCLCPP_DEBUG_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My debug message once per second after the second time %d", 4);

// C++ stream style
RCLCPP_DEBUG_STREAM_SKIPFIRST_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My debug message once per second after the second time " << 4);
```

## `stdout`への出力

Foxy以降では、ログ出力はデフォルトで`stderr`へ出力されるが、`$ export RCUTILS_LOGGING_USE_STDOUT=1`で`stdout`へ強制的に出力できる。

## 出力のバッファリング

`$ export RCUTILS_LOGGING_BUFFERED_STREAM=1`