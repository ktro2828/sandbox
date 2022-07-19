package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple group sample!")
	// Create checks
	ck1 := widget.NewCheck("check 1", nil)
	ck2 := widget.NewCheck("check 2", nil)
	// Create group from checks
	group := widget.NewGroup("Group", ck1, ck2)

	f := func() {
		var ret string = "result: "
		if ck1.Checked {
			ret += "Check-1 "
		}
		if ck2.Checked {
			ret += "Check-2"
		}
		label.SetText(ret)
	}
	button := widget.NewButton("OK", f)

	box := widget.NewVBox(label, group, button)

	window.SetContent(box)

	window.ShowAndRun()
	
}
