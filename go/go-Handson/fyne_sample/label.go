package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello!")

	label1 := widget.NewLabel("Hello Fyne!")
	label2 := widget.NewLabel("THis is a simple application!")

	box := widget.NewVBox(label1, label2)

	window.SetContent(box)

	window.ShowAndRun()
}
