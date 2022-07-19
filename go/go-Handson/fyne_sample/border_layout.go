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
	// Create buttons for each border
	btop := widget.NewButton("Top", nil)
	bbottom := widget.NewButton("Bottom", nil)
	bleft := widget.NewButton("Left", nil)
	bright := widget.NewButton("Right", nil)
	// Create boder layout
	border_layout := layout.NewBorderLayout(btop, bbottom, bleft, bright)
	// Create label
	label := widget.NewLabel("Center.")
	layout := fyne.NewContainerWithLayout(border_layout, btop, bbottom, bleft, bright, label)

	window.SetContent(layout)

	window.ShowAndRun()
}
