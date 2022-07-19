package main

import "fmt"

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

func main() {
	taro := Mydata{
		"Taro",
		[]int{10, 20, 30},
	}
	fmt.Println(taro)

	taro = rev1(taro)
	println("pass by value")
	fmt.Println(taro)
	
	println("pass by reference")
	rev2(&taro)
	fmt.Println(taro)
}

// 値渡しによってMydataを返す関数
func rev1(md Mydata) Mydata {
	var od []int = md.Data
	var nd []int
	for i := len(od) - 1; i >= 0; i-- {
		nd = append(nd, od[i])
	}
	md.Data = nd
	return md
}

// 参照渡しによって入力を直接変更する関数
func rev2(md *Mydata) {
	var od []int = (*md).Data
	var nd []int
	for i := len(od) - 1; i >= 0; i-- {
		nd = append(nd, od[i])
	}
	md.Data = nd
}
