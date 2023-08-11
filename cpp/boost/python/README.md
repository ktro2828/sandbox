# Boost Python

<!-- cspell: ignore libboost, lboost -->

`libboost_python`のシンボリックリンクを確認する。
下の場合、ビルド時に`-lboost_python310` or `-lboost_python3`でリンク可能。

```shell
$ ll /usr/lib/x86_64-linux-gnu/libboost_python3*
-rw-r--r-- 1 root root 657096  3月 17  2022 /usr/lib/x86_64-linux-gnu/libboost_python310.a
lrwxrwxrwx 1 root root     28  3月 17  2022 /usr/lib/x86_64-linux-gnu/libboost_python310.so -> libboost_python310.so.1.74.0
-rw-r--r-- 1 root root 243416  3月 17  2022 /usr/lib/x86_64-linux-gnu/libboost_python310.so.1.74.0
lrwxrwxrwx 1 root root     21  8月  7 08:43 /usr/lib/x86_64-linux-gnu/libboost_python3.so -> libboost_python310.so
```

そもそも、`libboost_python`がない場合は、以下でインストール
```shell
$ sudo apt update
$ sudo apt install libboost-python-dev
```

## ビルド
```shell
$ CPLUS_INCLUDE_PATH=/usr/include/python3.10 g++ -I`python3 -c 'from distutils.sysconfig import *; print get_python_inc()'` -DPIC -shared -fPIC -o basic.so basic.cpp -lboost_python310
```

- `CPLUS_INCLUDE_PATH=/usr/include/python3.10`が無いと、include errorになるので`bash`に`export`しておいててもいい。

    ```shell
    /usr/include/boost/python/detail/wrap_python.hpp:57:11: fatal error: pyconfig.h: No such file or directory
    57 | # include <pyconfig.h>
        |           ^~~~~~~~~~~~
    ```

- ``python3 -c 'from distutils.sysconfig import *; print(get_python_inc())'``ではincludeパスを指定している。実行してみると、以下のようになるので結果を直接書き込んでもいい。

    ```shell
    $ python3 -c 'from distutils.sysconfig import *; print(get_python_inc())'
    <string>:1: DeprecationWarning: The distutils package is deprecated and slated for removal in Python 3.12. Use setuptools or check PEP 632 for potential alternatives
    <string>:1: DeprecationWarning: The distutils.sysconfig module is deprecated, use sysconfig instead
    /usr/include/python3.10
    ```

## コンバータ
Boost.PythonにはC++<->Pythonの相互変換を行うためにコンバータという機能があり、ビルトインコンバータとして以下が定義されている。

| C++                            | Python  |
| :----------------------------- | :------ |
| bool                           | bool    |
| signed char / unsigned char    | int     |
| short / unsigned short         | int     |
| int / unsigned int             | int     |
| long / unsigned long           | int     |
| long long / unsigned long long | long    |
| float                          | float   |
| double                         | float   |
| long double                    | float   |
| char                           | str     |
| const char*                    | str     |
| std::string                    | str     |
| std::wstring                   | unicode |
| PyObject*                      | -       |
| std::complex                   | complex |

よって、`vector`や`array`、`map`等は別途コンバータを定義する必要がある。

## References
- [Boost.Python (日本語訳)](https://boostjp-python.readthedocs.io/ja/latest/index.html)
- [Boost.Pythonの機能をざっと紹介してみる](https://moriyoshi.hatenablog.com/entry/20091214/1260779899)