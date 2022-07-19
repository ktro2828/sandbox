package main

import "fmt"

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

func (md Mydata) PrintData() {
	fmt.Println("*** Mydata ***")
	fmt.Println("Name: ", md.Name)
	fmt.Println("Data: ", md.Data)
	fmt.Println("*** end ***")
}

func main() {
	var taro Mydata = Mydata{
		"Taro",
		[]int{109, 399, 28, 39},
	}
	taro.PrintData()
}
