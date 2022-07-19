package main

import "fmt"

func main() {
	var n int = 123
	var p *int = &n
	fmt.Println("number: ", n)
	fmt.Println("pointer: ", p)
	fmt.Println("value: ", *p)
}
