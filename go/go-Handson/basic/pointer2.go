package main

import "fmt"

func main() {
	var n int = 123
	var p *int = &n
	var m int = 10000
	var p2 *int = &m
	fmt.Printf("p value: %d, address: %p\n", *p, p)
	fmt.Printf("p2 value: %d, adsress: %p\n", *p2, p2)
	var ptmp *int = p
	// switch p and p2
	p = p2
	p2 = ptmp
	fmt.Printf("p value: %d, address: %p\n", *p, p)
	fmt.Printf("p2 value: %d, address: %p\n", *p2, p2)
}
