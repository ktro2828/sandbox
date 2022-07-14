package main

import "fmt"

func main() {
	// variable declaration
	var a int = 0
	var b = 1
	c := 2
	const d int = 3

	if a == 0 {
		println("a == 0")
	} else if a == 1 {
		println("a == 1")
	} else {
		fmt.Printf("a == %d\n", a)
	}
	fmt.Print(b)
	fmt.Print(c)
	fmt.Print(d)
}
