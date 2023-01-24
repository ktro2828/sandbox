package game

import (
	"bufio"
	"os"
)

const questionsCount = 60

func GetWords() ([]string, error) {
	path := "words.txt"
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}

	defer file.Close()

	lines := make(map[string]struct{})
	scanner := bufio.NewScanner(file)
	for scanner.Scan() {
		t := scanner.Text()
		lines[t] = struct{}{}
	}

	words := make([]string, 0)
	count := 0

	for k, _ := range lines {
		words = append(words, k)
		count++
		if count == questionsCount {
			break
		}
	}

	return words, nil

}
