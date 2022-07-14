package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
	"fyne.io/fyne/theme"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("Hello Fyne!")
	entry := widget.NewEntry()
	entry.SetText("0")
	button := widget.NewButton("Click me!", nil)
	box := widget.NewVBox(label, entry, button)
	window.SetContent(box)
	settings := myapp.Settings()
	settings.SetTheme(theme.LightTheme())
	window.ShowAndRun()
}
