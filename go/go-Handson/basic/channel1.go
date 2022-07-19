package main

import "fmt"

func total(n int, c chan int) {
	t := 0
	for i := 1; i <= n; i++ {
		t += i
	}
	// Set value to channel: <channel> <- VALUE
	c <- t
}


func main() {
	// Create channel that is type of int
	c := make(chan int)
	go total(100, c)
	// Get value from channel: VARIABL <-<channel>
	fmt.Println("total: ", <-c)
}
