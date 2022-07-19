package main

import (
	"fmt"
	"strconv"
)

type intp int

func (num intp) IsPrime() bool {
	n := int(num)
	for i := 2; i <= (n / 2); i++ {
		if n % i == 0 {
			return false
		}
	}
	return true
}

func (num intp) PrimeFactor() []int {
	var arr []int
	x := int(num)
	n := 2
	for x > n {
		if x % n == 0 {
			x /= n
			arr = append(arr, n)
		} else {
			if n == 2 {
				n++
			} else {
				n += 2
			}
		}
	}
	arr = append(arr, x)
	return arr
}

func main() {
	s := "12"
	n, _ := strconv.Atoi(s)
	x := intp(n)
	fmt.Printf("%d [%t].\n", x, x.IsPrime())
	fmt.Println(x.PrimeFactor())
	x *= 2
	fmt.Printf("%d [%t].\n", x, x.IsPrime())
	fmt.Println(x.PrimeFactor())
}
