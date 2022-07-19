package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
	"strconv"
)


func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple slider!")
	slider := widget.NewSlider(0.0, 100.0)
	f := func() {
		label.SetText("value: " + strconv.Itoa(int(slider.Value)))
	}
	button := widget.NewButton("Check", f)
	box := widget.NewVBox(label, slider, button)
	window.SetContent(box)

	window.ShowAndRun()
}
