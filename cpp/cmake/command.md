# CMake Commands

## REFERENCES

- [cmake-commands](https://cmake.org/cmake/help/latest/manual/cmake-commands.7.html)

## [`cmake_minimum_required`](https://cmake.org/cmake/help/latest/command/cmake_minimum_required.html)

利用可能なcmakeの最低バージョンを指定する。内部で暗黙的に[`cmake_policy(VERSION)`](https://cmake.org/cmake/help/latest/command/cmake_policy.html#version)コマンドを呼び出している。

`cmake_minimum_required(VERSION <min> [...<policy_max> [FATAL_ERROR]])`

## [`project`](https://cmake.org/cmake/help/latest/command/project.html)

プロジェクト名を指定し、`PROJECT_NAME`変数に代入する。トップレベルの`CMakeLists.txt`から指定した場合、`CMAKE_PROJECT_NAME`にも代入される。

`project(<PROJECT-NAME> [VERSION <major>[.<minor>[.<patch>][.<tweak>]]]) [DESCRIPTION <project-descritpion>] [HOMEPAGE_URL <url>] [LANGUAGES <language-name>...]`

## [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html)

指定されたパッケージを検索する。

`find_package(<package-name> [<version>] [REQUIRED] [COMPONENTS <components>...])`

## [`add_executable`](https://cmake.org/cmake/help/latest/command/add_executable.html)

指定されたソースファイルをexecutableとしてプロジェクトに追加する。

`add_executable(<name> <options>... <sources>...)`

## [`add_library`](https://cmake.org/cmake/help/latest/command/add_library.html)

指定されたソースファイルをライブラリとしてプロジェクトに追加する。

`add_library(<name> [<type>] [EXCLUDE_FROM_ALL] <sources>...)`

- `<type>`
  - `STATIC`: 他のターゲットをリンクする際に使うオブジェクトファイルのアーカイブ
  - `SHARED`: 他のターゲットにリンクまたは実行時に読み込まれる動的ライブラリ
  - `MODULE`: 他のターゲットにリンクされない可能性があるが、`dlopen`のような機能を使用して実行時に動的に読み込まれる可能性があるプラグイン

## [`include_directories`](https://cmake.org/cmake/help/latest/command/include_directories.html)

指定されたディレクトリをコンパイラがインクルードファイルを検索するためのパス(=`${INCLUDE_DIRECTORIES}`)に追加する。
相対パスは`${CMAKE_CURRENS_SOURCE_DIR}`からの相対パスとして解釈される。

`include_directories([AFTER|BEFORE] [SYSTEM] <dir1> [<dir2>...])`

- `[AFTER|BEFORE]`
  - 指定されたディレクトリは現在の検索リスト後方に追加される。`CMAKE_INCLUDE_DIRECTORIES_BEFORE`が`ON`の場合、前方に追加される。
  - `AFTER|BEFORE`を指定することで、後方に追加するか、前方に追加するかを明示的に指定できる。
- `SYSTEM`: このオプションを使った場合、指定されたディレクトリはOS特有のインクルードディレクトリがルートパスとして解釈される。

> [!NOTE]
> 特定のターゲットに対するインクルードパスを追加したい場合は、`target_include_directories()`コマンドが推奨

## [`target_include_directories`](https://cmake.org/cmake/help/latest/command/target_include_directories.html)

ターゲットに対するインクルードパスを追加する。`<target>`は事前に`add_executable()`や`add_library()`で宣言しておく必要がある。

`target_include_directories(<target> [SYSTEM] [AFTER|BEFORE] <INTERFACE|PUBLIC|PRIVATE> [items1...]) [<INTERFACE|PUBLIC|PRIVATE> [items2...]...])`

## [`install`](https://cmake.org/cmake/help/latest/command/install.html)

### Usage

※ `DESTINATION`はunixの場合、`/usr/local/`がプレフィックス

1. ファイル|ディレクトリのインストール

- `install(FILES|DIRECTORY <file1|dir1> <file2|dir2> DESTINATION <dir> [OPTION1] [OPTION2] ...)`
  - `FILES｜DIRECTORY`: インストールするファイル|ディレクトリの指定
  - `DESTINATION`: ファイル|ディレクトリをインストールするディレクトリを指定
  - `RENAME, optional`: インストール先のファイル|ディレクトリ名を変更する
  - `COMPONENT, optional`: 特定のコンポーネントがインストールされるときにの`FILES|DIRECTORY`をインストールする
  - `PERMISSIONS, optional`: インストールするファイル|ディレクトリの権限を指定する
  - `OPTIONAL, optional`: インストールするファイル|ディレクトリが存在しない場合エラーを吐かない

2. ターゲットのインストール

- `install(TARGETS <target1> <target2> ... <artifact-kind> DESTINATION <dir> [OPTION1] [OPTION2] ...)`
  - `TARGETS`: インストールするターゲット(e.g. 実行ファイル、ライブラリ)
  - `artifact-kind`: ターゲットの種類(`[ARCHIVE|RUNTIME|FRAMEWORK|BUNDLE|PRIVATE_HEADER|PUBLIC_HEADER|RESOURCE]`)
  - `DESTINATION`: インストール先のディレクトリ名

- `artifact-kind`
  - `ARCHIVE`: 静的ライブラリ
  - `LIBRARY`: 共有ライブラリ
  - `RUNTIME`: 実行ファイル
  - `OBJECTS`: オブジェクトファイル

3. エクスポートセットのインストール

- `install(EXPORT <export-set> DESTINATION <dir> [OPTION1] [OPTION2] ...)`
  - `EXPORT`: エクスポートセットを指定
  - `DESTINATION`: インストール先の指定

### Example

1. `my_config*.conf`を`/usr/local/etc/my_project`にインストールする例:

```cmake
install(FILES my_config1.conf my_config2.conf DESTINATION etc/my_project)
```

2. `my_executable`を`/usr/local/bin`に`my_library`を`/usr/local/lib`にインストールする例:

```cmake
install(TARGETS my_executable
        RUNTIME DESTINATION bin)
install(TARGETS my_library
        LIBRARY DESTINATION lib)
```

3. ライブラリ my_library をエクスポートセット MyLibraryTargets に含めてインストールする例:
```cmake
cmake_minimum_required(VERSION 3.14)
project(MyProject VERSION 1.0)

add_library(my_library SHARED src/my_library.cpp)

# my_libraryをlibディレクトリにインストールし、同時にエクスポートセットMyLibraryTargetsに追加
install(TARGETS my_library
  EXPORT MyLibraryTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# エクスポートセットのインストール
# FILE: エクスポートセットを保存するファイル名
# NAMESPACE: エクスポートされたターゲットに名前空間を追加
# DESTINATION: エクスポートセットをインストールするディレクトリ名
install(EXPORT MyLibraryTargets
  FILE MyLibraryTargets.cmake
  NAMESPACE MyLibrary::
  DESTINATION lib/cmake/MyProject
)
```

これによって、他のプロジェクトでエクスポートされた`my_library`を使用可能:
```cmake
cmake_minimum_required(VERSION 3.14)
project(AnotherProject)

find_package(MyProject REQUIRED)

add_executable(my_executable src/main.cpp)
target_link_libraries(my_executable MyLibrary::my_library)
```

## [`include`](https://cmake.org/cmake/help/latest/command/include.html)

### Usage

- `include(<file|module>) [OPTIONAL] [NO_POLICY_SCOPE]`
  - `<file|module>`: インクルードするファイルまたはモジュールの名前
  - `OPTIONAL`: 指定したファイルが存在しない場合でもエラーにしない
  - `NO_POLICY_SCOPE`: インクルードしたファイル内でのポリシー設定を親スコープに影響させない。

```cmake
# 存在しないファイルでもエラーを吐かない
include(<NON_EXISTING_FILE>.cmake OPTIONAL)
```

1. 標準モジュールのインクルード

CMakeのビルトインモジュールを読み込むことができる。

```cmake
# 例: CMakePackageConfigHelperのインクルード
include(CMakePackageConfigHelper)
```

2. カスタムモジュールのインクルード

プロジェクト独自のCMakeスクリプトファイルをインクルードすることもできる。

```cmake
include(${CMAKE_CURRENT_SOURCE_DIR}/<modulename>.cmake)
```

### Example

```cmake : my_module.cmake
# my_module.cmake

# 関数を定義
function(my_custom_function)
    message("This is a custom function")
endfunction()

# 変数を定義
set(MY_CUSTOM_VARIABLE "Hello, world!")
```

```cmake : CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(my_project)

# my_module.cmakeをインクルード
include(${CMAKE_CURRENT_SOURCE_DIR}/my_module.cmake)

# インクルードしたモジュールの関数を使用
my_custom_function()

# インクルードしたモジュールの変数を使用
message("Custom variable: " ${MY_CUSTOM_VARIABLE})
```
