package main

import "fmt"

func main() {
	var n int = 123
	var p *int = &n
	var q **int = &p
	var m int = 1000
	var p2 *int = &m
	var q2 **int = &p2

	fmt.Printf("q value: %d, address: %p\n", **q, *q)
	fmt.Printf("q2 value: %d, address: %p\n", **q2, *q2)

	// switch p and p2
	var ptmp *int = p
	p = p2
	p2 = ptmp

	fmt.Printf("q value: %d, address: %p\n", **q, *q)
	fmt.Printf("q2 value: %d, address: %p\n", **q2, *q2)
}
