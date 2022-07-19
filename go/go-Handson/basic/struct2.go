package main

import "fmt"

// Mydata is structure. <- このコメントがないとVScodeで"exported Mydata should have commnet or be unexported"とでる
type Mydata struct {
	Name string
	Data []int
}

func main() {
	var taro Mydata = Mydata{"Taro", []int{10, 20, 30}}
	var hanako Mydata = Mydata{
		Name: "Hanako",
		Data: []int{90, 80, 70},
	}
	fmt.Println(taro)
	fmt.Println(hanako)
}
