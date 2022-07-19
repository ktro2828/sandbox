package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	b1 := widget.NewButton("One", nil)
	b2 := widget.NewButton("Two", nil)
	b3 := widget.NewButton("Three", nil)
	b4 := widget.NewButton("Four", nil)
	b5 := widget.NewButton("Five", nil)
	b6 := widget.NewButton("Six", nil)
	b7 := widget.NewButton("Seven", nil)
	b8 := widget.NewButton("Eight", nil)
	b9 := widget.NewButton("Nine", nil)
	b10 := widget.NewButton("Ten", nil)
	box := widget.NewVBox(b1, b2, b3, b4, b5, b6, b7, b8, b9, b10)
	container := widget.NewScrollContainer(box)
	window.SetContent(container)

	window.ShowAndRun()
}
