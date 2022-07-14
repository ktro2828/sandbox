package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple app to create check box!")

	f := func(f bool) {
		if f {
			label.SetText("CHECKED!")
		} else {
			label.SetText("not checked.")
		}
	}
	check := widget.NewCheck("Check", f)
	check.SetChecked(true)

	box := widget.NewVBox(label, check)

	window.SetContent(box)
	window.ShowAndRun()
}
