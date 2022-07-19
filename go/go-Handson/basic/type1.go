package main

func type_control() {
	var sum_int int
	sum_int = 5 + 6 + 3
	var sum_float float32 = float32(sum_int)
	var avg float32
	avg = sum_float / 3.0
	if avg > 4.5 {
		println("good")
	}
}
