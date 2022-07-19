package main

import (
	"fmt"
	"strings"
	"strconv"
)


// Data is interface for Mydata.
type Data interface {
	SetValue(vals map[string]string)
	PrintData()
}

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

// SetValue is method for Mydata.
func (md *Mydata) SetValue(vals map[string]string) {
	md.Name = vals["name"]
	valt := strings.Split(vals["data"], " ")
	vali := []int{}
	for _, i := range valt {
		n, _ := strconv.Atoi(i)
		vali = append(vali, n)
	}
	md.Data = vali
}

// PrintData is method for Mydata.
func (md *Mydata) PrintData() {
	fmt.Println("Name: ", md.Name)
	fmt.Println("Data: ", md.Data)
}

// Yourdata is structure.
type Yourdata struct {
	Name string
	Mail string
	Age int
}

// SetValue is method for Yourdata.
func (yd *Yourdata) SetValue(vals map[string]string) {
	yd.Name = vals["name"]
	yd.Mail = vals["mail"]
	n, _ := strconv.Atoi(vals["age"])
	yd.Age = n
}

// Printdata is method for Yourdata.
func (yd *Yourdata) PrintData() {
	fmt.Printf("I'm %s. (%d).\n", yd.Name, yd.Age)
	fmt.Printf("mail: %s.\n", yd.Mail)
}

func main() {
	ob := [2]Data{}
	ob[0] = new(Mydata)
	md := map[string]string{
		"name": "Sachiko",
		"data": "55, 66, 77",
	}
	ob[0].SetValue(md)
	ob[1] = new(Yourdata)
	yd := map[string]string{
		"name": "Mami",
		"mail": "mami@mume.mo",
		"age": "34",
	}
	ob[1].SetValue(yd)
	for _, d := range ob {
		d.PrintData()
		fmt.Println()
	}
}
