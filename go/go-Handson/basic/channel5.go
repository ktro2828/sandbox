package main

import "fmt"

func total(cs chan int, cr chan int) {
	fmt.Print("...waiting for cs\n")
	n := <-cs
	fmt.Println("n = ", n)
	t := 0
	for i := 1; i <= n; i++ {
		t += 1
	}
	cr <- t
}

func main() {
	cs := make(chan int)
	cr := make(chan int)
	go total(cs, cr)
	fmt.Print("Set cs value!\n")
	cs <- 100
	fmt.Println("total:", <-cr)
}
