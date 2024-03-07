# Pythonでのマルチタスク処理

- [並行処理 (concurrent computing)](https://ja.wikipedia.org/wiki/%E4%B8%A6%E8%A1%8C%E8%A8%88%E7%AE%97)
- [並列処理 (parallel computing)](https://ja.wikipedia.org/wiki/%E4%B8%A6%E5%88%97%E8%A8%88%E7%AE%97)

Pythonでマルチタスク処理を行う方法としては以下の2パターンが存在する
- 1プロセス内で複数スレッドを立ち上げて実行 -> `threading`
- 複数プロセスをそれぞれ1スレッドごと立ち上げて実行 -> `multiprocessing`

## threading

Pythonの標準ライブラリの一つで、`__thread`という低水準のモジュールをカプセル化したモジュール。

Unix系のOSではスレッド周りで主に以下のシステムコール関数が使える。

|   System Call   | Description                           |
| :-------------: | :------------------------------------ |
|     start()     | Start the thread                      |
|    setName()    | Name the thread                       |
|    getName()    | Get the name of thread                |
| setDaemon(True) | Make a thread a daemon                |
|     join()      | Wait until thread finishes processing |
|      run()      | Execute thread processing manually    |

Pythonのスレッドはプロセスでシミュレートしたものではなく、OSの実際のスレッド利用する。
UnixやLinux系では[POSIXスレッド](https://ja.wikipedia.org/wiki/POSIX%E3%82%B9%E3%83%AC%E3%83%83%E3%83%89)を、WindowsではWindowsスレッドを利用する。

デーモンスレッドとして設定した場合、そのスレッドはMainスレッドが終了するときに終了する。

```python
from threading import Thread, current_thread, active_count
import time

def run(task: str) -> None:
    cur_thread = current_thread()
    print(f"{task=} (thread name: {cur_thread.name})")
    time.sleep(1)
    print(f"{task=} (2s)")
    time.sleep(1)
    print(f"{task=} (1s)")
    time.sleep(1)
    print(f"{task=} (0s)")


def main():
    t1 = Thread(target=run, args=("t1", ))
    t2 = Thread(target=run, args=("t2", ), name="Thread T2") # invoke setName()

    # start()
    t1.start()
    t2.start()

    print(f"Active threads: {active_count()}") # 3 = t1 + t2 + Main

    # join()
    t1.join()
    t2.join()

    # wait until t1 and t2 finish processing
    cur_thread = current_thread()
    print(cur_thread.name)
    print(f"Active threads: {active_count()}") # 1 = Main

if __name__ == "__main__":
    main()
```

## multiprocessing

Unix系OSでは`fork()`というシステムコールでプロセスを作成できる。
`fork()`を呼び出すと現在のプロセスがコピーされ子プロセスを生成する。
子プロセスからは`getppid()`で親プロセスのプロセスIDを取得できる(`getppid()` = Get Parent Process ID)。

Pythonの`os`モジュールでは、システムコール系がカプセル化されており`fork()`も呼び出すことができる。
Windowsでは`fork()`システムコールが無いが、`multiprocessing`は擬似的に`fork()`処理を行うことでOSに依存せずプロセスを作成することができる。
また`multiprocessing`モジュールは並列処理可能なモジュールで、`threading`はGILのせいで並列処理ができないため、`multiprocessing`が実装された背景がある。
`multiprocessing`でプロセスを作る際は、親プロセスのすべてのPythonオブジェクトを`pickle`でシリアライズ後、子プロセスに渡す仕組み担っている。

```python
from multiprocessing import Process
import os

def run_proc(name: str):
    print(f"Run chile process {name} ({os.getpid()})")


def main():
    print(f"Parent process {os.getpid()}")
    p = Process(target=run_proc, args=("child1",))
    # start()
    p.start()
    # join()
    p.join()

if __name__ == "__main__":
    main()
```


## REFERENCES

- [並行実行](https://docs.python.org/ja/3/library/concurrency.html)
- [threading --- スレッドベースの並列処理](https://docs.python.org/ja/3/library/threading.html)
- [multiprocessing --- プロセスベースの並列処理](https://docs.python.org/ja/3/library/multiprocessing.html)
- [threadingとmultiprocessing](https://zenn.dev/kaitolucifer/articles/1f0eda0ca1ed52)
