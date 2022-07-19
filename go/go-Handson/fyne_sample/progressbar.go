package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)


func main() {
	var val float64 = 0.0
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple progeressbar!")
	pbar := widget.NewProgressBar()
	f := func() {
		val += 0.1
		if val > 1.0 {
			val = 0.0
		}
		pbar.SetValue(val)
	}
	button := widget.NewButton("Up!", f)
	box := widget.NewVBox(label, pbar, button)
	window.SetContent(box)
	window.ShowAndRun()
}
