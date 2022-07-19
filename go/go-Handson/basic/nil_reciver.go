package main

import (
	"fmt"
	"strings"
	"strconv"
)

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

// SetValue is method for Mydata.
func (md *Mydata) SetValue(vals map[string]string) {
	md.Name = vals["name"]
	var valt []string = strings.Split(vals["data"], " ")
	var vali []int
	for _, i := range valt {
		n, _ := strconv.Atoi(i)
		vali = append(vali, n)
	}
	md.Data = vali
}

// PrintData is method for Mydata.
func (md *Mydata) PrintData() {
	if md != nil {
		fmt.Println("Name: ", md.Name)
		fmt.Println("Data: ", md.Data)
	} else {
		fmt.Println("**This is Nil value.**")
	}
}

func main() {
	var ob *Mydata
	ob.PrintData()

	ob = &Mydata{}
	ob.PrintData()

	md := map[string]string{
		"name": "Jiro",
		"data": "123 456 789",
	}
	ob.SetValue(md)
	ob.PrintData()
}
