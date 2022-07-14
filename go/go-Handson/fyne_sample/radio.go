package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple radio app!")
	keys := []string{"One", "Two", "Three"}

	f := func(s string) {
		if s == "" {
			label.SetText("not selected.")
		} else {
			label.SetText("selected: " + s)
		}
	}

	radio := widget.NewRadio(keys, f)

	radio.SetSelected("One")
	box := widget.NewVBox(label, radio)
	window.SetContent(box)

	window.ShowAndRun()
}
