package main

import "fmt"

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

func main() {
	var taro *Mydata = new(Mydata)
	fmt.Println(*taro)
	taro.Name = "Taro"
	taro.Data = []int{10, 20, 30}
	fmt.Println(*taro)	
}
