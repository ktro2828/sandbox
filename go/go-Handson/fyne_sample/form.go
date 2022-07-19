package main

import (
	"fyne.io/fyne/app"
	"fyne.io/fyne/widget"
)

func main() {
	myapp := app.New()
	window := myapp.NewWindow("Hello")
	label := widget.NewLabel("This is a simple form!")

	// Form entry
	name_entry := widget.NewEntry()
	pwd_entry := widget.NewPasswordEntry()
	// Create form item from entry
	name_fitem := widget.NewFormItem("Name", name_entry)
	pwd_fitem := widget.NewFormItem("Pass", pwd_entry)
	// Create form
	form := widget.NewForm(name_fitem, pwd_fitem)

	f := func() {
		label.SetText(name_entry.Text + " & " + pwd_entry.Text)
	}
	button := widget.NewButton("OK", f)

	box := widget.NewVBox(label, form, button)

	window.SetContent(box)
	window.ShowAndRun()
}
