# CMake Builtin Modules

各モジュールは、はじめに`include(<modulename>)`でインクルードしてから使う。

## [`CMakePackageConfigHelpers`](https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html)

`CMakePackageConfigHelpers`は、CMakeでパッケージ構成ファイル（Configファイル）を作成および管理するためのモジュール。
このモジュールを使用すると、他のプロジェクトが`find_package`コマンドを使って簡単にパッケージを見つけられるようになる。

### Usage

1. [`configure_package_config_file`](https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#command:configure_package_config_file)

プロジェクトのコンフィグファイルを生成する。

`configure_package_config_file(<input> <output> INSTALL_DESTINATION <path> ...)`

```cmake
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/MyConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/MyProjectConfig.cmake"
  INSTALL_DESTINATION lib/cmake/MyProject
)
```

- `MyConfig.make.in`: テンプレートファイル名
- `MyProjectConfig.cmake`: 生成されるコンフィグファイル
- `INSTALL_DESTINATION`: コンフィグファイルのインストール先ディレクトリ

```cmake : MyConfig.cmake.in
# MyConfig.cmake.in
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)
include("${CMAKE_CURRENT_LIST_DIR}/MyLibraryTargets.cmake")
```

2. [`write_basic_package_version_file`](https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#command:write_basic_package_version_file)

パッケージのバージョン情報を含むファイルを生成する。`find_package`コマンドがバージョンをチェックするために使用される。

`write_basic_package_version_file(<filename> [VERSION] COMPATIBILITY <AnyNewerVersion|SameMajorVersion|SameMinorVersion|ExactVersion> [ARCH_INDEPENDENT])`

```cmake
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/MyProjectConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
```

- `VERSION`: プロジェクトのバージョンを指定(`project(<project-name> VERSION <version>)`で`${PROJECT_VERSION}`を決定する必要がある)
- `COMPATIBILITY`: バージョンの互換性ルールを指定(`AnyNewerVersion`は指定したバージョン移行の任意のバージョンを許容)

## [`FetchContent`](https://cmake.org/cmake/help/latest/module/FetchContent.html#fetchcontent)

`FetchContent`はCMakeで外部プロジェクトは簡単に取得し、ビルドプロセスに組み込むためのモジュール。

1. [`FetchContent_Declare`](https://cmake.org/cmake/help/latest/module/FetchContent.html#command:fetchcontent_declare)

外部コンテンツのダウンロード方法と設定を宣言する。

`FetchContent_Declare(<name> <contentOptions> [EXCLUDE_FROM_ALL] [SYSTEM] [OVERRIDE_FIND_PACKAGE | FIND_PACKAGE_ARGS args ...])`

```cmake
include(FetchContent)

FetchContent_Declare(
  <name>
  URL <url>
  URL_HASH <hash>
  GIT_REPOSITORY <git-repository-url>
  GIT_TAG <git-tag>
  GIT_SHALLOW <bool>
  SOURCE_DIR <source-dir>
  BINARY_DIR <binary-dir>
  ...
)
```

- `<name>`: 外部コンテンツの名前(任意の名前でOK)
- `URL`: ダウロードするアーカイブファイルのURL
- `URL_HASH`: アーカイブのハッシュ値(SHA256など)
- `GIT_REPOSITORY`: GitリポジトリのURL
- `GIT_TAG`: Gitリポジトリの特定のタグまたはコミット
- `GIT_SHALLOW`: 浅いクローンを行うかどうか(=サブモジュールもクローンするかどうか)

2. [`FetchContent_MakeAvailable`](https://cmake.org/cmake/help/latest/module/FetchContent.html#command:fetchcontent_makeavailable)

`FetchContent_Declare`で宣言されたコンテンツをダウンロードし、利用可能にする。

`FetchContent_MakeAvailable(<name1> [<name2> ...])`

```cmake
include(FetchContent)

# Fetch GoogleTest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.10.0
)

# Fetch fmt library
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG 7.1.3
)

# 複数のコンテンツを同時に指定することでそれらを一度に利用可能にする
FetchContent_MakeAvailable(googletest fmt)
```
