package main

import "fmt"

func main() {
	arr := []int{10, 20, 30}
	fmt.Println(arr)
	initialize(&arr)
	fmt.Println(arr)	
}

func initialize(arr *[]int) {
	for i := 0; i < len(*arr); i++ {
		(*arr)[i] = 0
	}
}
