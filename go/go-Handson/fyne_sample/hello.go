package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello!")

	label := widget.NewLabel("Hello Fyne!")
	window.SetContent(label)

	window.ShowAndRun()
}
