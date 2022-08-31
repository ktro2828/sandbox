package main

/**
  スライスの宣言
  var x []int

  宣言 + 初期化
  var x = []int{1, 2, 3, 4}
  x := []int{1, 2, 3, 4}
 */

import "fmt"

func main() {
	// スライス型の定義
	var a = []int{1, 2, 3, 4}
	var x []int

	x = append(x, 1)

	// 長さ4, 容量10のスライスを0埋めで生成
	y := make([]int, 4, 10)

	fmt.Println(a)
	fmt.Println(x)
	fmt.Println(y)
}
