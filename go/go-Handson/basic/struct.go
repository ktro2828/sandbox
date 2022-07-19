package main

import "fmt"


type Human struct {
	name string
	age int
}

func (h Human) greeting() {
	fmt.Println("Hi, I'm " + h.name)
}


func main() {
	bob := Human{"Bob", 18}
	fmt.Println(bob.name)
	bob.greeting()
}
