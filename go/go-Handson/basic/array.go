package main

/**
  配列の宣言
  var x [10]int

  宣言 + 初期化
  var x = [3]int{1, 2, 3}
  x := [3]int{1, 2, 3}
 */


import "fmt"

func main() {
	// array型の定義
	var a [4]string
	a[0] = "hoge"

	// array型の定義 + 初期値指定
	var a2 = [2]string{"hoge", "fuga"}
	b := [4]int{1, 2, 3, 4}

	// 組み込みのprintln()だとError
	fmt.Println(a)
	fmt.Println(b)
	fmt.Println(a2)
}
