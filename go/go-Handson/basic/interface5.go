package main

import (
	"fmt"
)


// General is all type data.
type General interface{}


// GData is holding General value.
type GData interface {
	Set(nm string, g General)
	Print()
}


// GDataImpl is structure.
type GDataImpl struct {
	Name string
	Data General
}

// Set is method for GDataImpl.
func (gd *GDataImpl) Set(nm string, g General) {
	gd.Name = nm
	gd.Data = g
}


// Print is method for GDataImpl.
func (gd *GDataImpl) Print() {
	fmt.Printf("<<%s>> ", gd.Name)
	fmt.Println(gd.Data)
}


func main() {
	var data = []GDataImpl{}
	data = append(data, GDataImpl{"Taro", 123})
	data = append(data, GDataImpl{"Hanako", "hello!"})
	data = append(data, GDataImpl{"Sachikof", []int{123, 456, 789}})
	for _, ob := range data {
		ob.Print()
	}
}
