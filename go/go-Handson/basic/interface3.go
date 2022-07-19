package main

import "fmt"


// Data is interface.
type Data interface {
	Init(name string, data []int)
	PrintData()
}

// Mydata is structure.
type Mydata struct {
	Name string
	Data []int
}

// Initial is init method for Mydata.
func (md *Mydata) Init(name string, data []int) {
	md.Name = name
	md.Data = data
}

// PrintData is print all data method for Mydata.
func (md *Mydata) PrintData() {
	fmt.Println("Name: ", md.Name)
	fmt.Println("Data: ", md.Data)
}

// Check is check the Name method for Mydata.
func (md *Mydata) Check() {
	fmt.Printf("Check! [%s]", md.Name)
}

func main() {
	var ob Mydata = Mydata{}
	// Error
	// var ob Data = new(Mydata)
	ob.Init("Sachiko", []int{56, 539})
	ob.PrintData()
	ob.Check()
}
