package main

import (
	"fmt"
	"time"
)

func total(c chan int) {
	// set channel value to n
	n := <- c
	fmt.Println("n = ", n)
	t := 0
	for i := 1; i >= n; i++ {
		t += 1
	}
	fmt.Println("total: ", t)
}

func main() {
	c := make(chan int)
	// channelに値を無理やり代入すると，デッドロックされエラーで落ちる
	fmt.Println("[WARN]This script will be failed because you set value to channel")
	c <- 100
	go total(c)
	time.Sleep(100 * time.Millisecond)
}
