# 「江添亮のC++入門」
Github: https://ezoeryou.github.io/cpp-intro/
Web: https://cpp.rainy.me/

## GCC: C++コンパイラー

- `-std`: C++の規格選択オプション．
- `-Wall`: コンパイラの警告メッセージのほぼすべてを有効にするオプション．
- `--pedantic-errors`: C++の規格を厳格に守るオプション．規格違反コードはコンパイルエラー．

```shell
g++ -std=c++17 -Wall --pedanric-errors -o OUTPUT SOUCE.cpp
```

- GCCマニュアル参照方法

```shell
man gcc
```
