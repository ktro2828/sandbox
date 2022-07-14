package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
	"strconv"
)


func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")

	label := widget.NewLabel("This is a simple app to count the number of clicks!")
	var cnt int = 0
	// define function and set it as a variable
	f := func() {
		cnt++
		label.SetText("Count: " + strconv.Itoa(cnt))
	}
	// add content
	button := widget.NewButton("Click me!", f)
	box := widget.NewVBox(label, button)
	window.SetContent(box)

	// show
	window.ShowAndRun()
}
