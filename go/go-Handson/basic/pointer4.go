package main

import "fmt"

func main() {
	var n int = 123
	fmt.Printf("Original value: %d\n", n)
	change1(n)
	fmt.Printf("After change1() value: %d\n", n)
	change2(&n)
	fmt.Printf("After change2() value: %d\n", n)
}

func change1(n int) {
	n *= 2
}

func change2(n *int) {
	*n *= 2
}
