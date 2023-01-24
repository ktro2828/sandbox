package game

import (
	"bufio"
	"fmt"
	"io"
	"os"
)

type Game struct {
	OutStream, ErrStream io.Writer
}

func (g *Game) Run() int {
	sch := start(os.Stdin)
	<-sch
	return 0
}

func start(r io.Reader) <-chan struct{} {
	fmt.Println("Start Typing Game")
	fmt.Println("Time limits is 30[s]")
	fmt.Println(">>> Press any key to start")
	ch := make(chan struct{})
	go func() {
		s := bufio.NewScanner(r)
		for s.Scan() {
			ch <- struct{}{}
			break
		}
	}()
	return ch
}

func input(r io.Reader) <-chan string {
	ch := make(chan string)
	go func() {
		s := bufio.NewScanner(r)
		for s.Scan() {
			ch <- s.Text()
		}
		close(ch)
	}()
	return ch
}
