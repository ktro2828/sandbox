package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	l1 := widget.NewLabel("This is First tab item")
	l2 := widget.NewLabel("This is Second tab item")
	titem1 := widget.NewTabItem("First", l1)
	titem2 := widget.NewTabItem("Second", l2)
	tc := widget.NewTabContainer(titem1, titem2)
	box := widget.NewVBox(tc)
	window.SetContent(box)
	window.ShowAndRun()
}
