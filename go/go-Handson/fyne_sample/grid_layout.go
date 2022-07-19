package main

import (
	"fyne.io/fyne"
	"fyne.io/fyne/app"
	"fyne.io/fyne/layout"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	// Create grid layout
	grid_layout := layout.NewGridLayout(3)
	// Create buttons
	b1 := widget.NewButton("One", nil)
	b2 := widget.NewButton("Two", nil)
	b3 := widget.NewButton("Three", nil)
	b4 := widget.NewButton("Four", nil)
	b5 := widget.NewButton("Five", nil)
	b6 := widget.NewButton("Six", nil)
	b7 := widget.NewButton("Seven", nil)
	b8 := widget.NewButton("Eight", nil)
	b9 := widget.NewButton("Nine", nil)
	b10 := widget.NewButton("Ten", nil)
	// Create spacer
	spacer := layout.NewSpacer()
	layout := fyne.NewContainerWithLayout(
		grid_layout,
		b1, b2, b3, b4, spacer,
		b5, b6, spacer,
		b7, b8, b9, b10,
	)
	window.SetContent(layout)

	window.ShowAndRun()
}
