package main

/**
  複数のスレッドで共有メモリの同時アクセスをsync.Mutexを使って制御する
 */

import (
	"sync"
	"fmt"
	"time"
	"strconv"
)

// SrData is struct
type SrData struct {
	msg string
	mux sync.Mutex
}

func main() {
	sd := SrData{msg: "Start"}
	// function to print message
	prmsg := func(nm string, n int) {
		fmt.Println(nm, sd.msg, ": sleep for ", n, "[ms]")
		time.Sleep(time.Duration(n) * time.Millisecond)
	}

	main := func(n int) {
		const nm string = "*main"
		sd.mux.Lock() // 共有メモリをLock
		fmt.Print("[*main] Lock\n")
		for i := 0; i < 5; i++ {
			sd.msg += " m" + strconv.Itoa(i)
			prmsg(nm, 100)
		}
		sd.mux.Unlock() // 共有メモリをUnLock
		fmt.Print("[*main] UnLock\n")
	}

	hello := func(n int) {
		const nm string = "hello"
		sd.mux.Lock() // 共有メモリをLock
		fmt.Print("[hello] Lock\n")
		for i := 0; i < 5; i++ {
			sd.msg += " h" + strconv.Itoa(i)
			prmsg(nm, n)
		}
		sd.mux.Unlock(); // 共有メモリをUnLock
		fmt.Print("[hello] UnLock\n")
	}

	// go-routine
	go main(100)
	go hello(50)
	time.Sleep(5 * time.Second)
}
