package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
	"strconv"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello!")
	label := widget.NewLabel("This is a simple app to count the total of entered number!")
	entry := widget.NewEntry()
	entry.SetText("0")
	f := func() {
		n, _ := strconv.Atoi(entry.Text)
		label.SetText("Total: " + strconv.Itoa(total(n)))
	}
	button := widget.NewButton("Click me!", f)

	box := widget.NewVBox(label, entry, button)

	window.SetContent(box)

	window.ShowAndRun()	
}

func total(n int) int {
	t := 0
	for i := 0; i <= n; i++ {
		t += i
	}
	return t
}
