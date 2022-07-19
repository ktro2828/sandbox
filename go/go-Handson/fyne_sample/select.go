package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)


func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple choice app!")
	check_list := []string{"Eins", "Twei", "Drei"}
	f := func(s string) {
		label.SetText("selected: " + s)
	}
	slct := widget.NewSelect(check_list, f)
	box := widget.NewVBox(label, slct)
	window.SetContent(box)
	window.ShowAndRun()
}
