/*
converter is a package to convert image extension.
*/

package converter

import (
	"errors"
	"fmt"
	"image"
	"image/jpeg"
	"image/png"
	"io"
	"io/ioutil"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// ConvertExt converts image files to specified extension in the src directory.
func ConvertExt(dir string, from string, to string) (int, error) {
	from = strings.ToLower(from)
	to = strings.ToLower(to)

	if err := validateArgs(from, to); err != nil {
		return 0, nil
	}

	fileNames := make(chan string)
	go func() {
		globDir(dir, from, fileNames)
	}()

	var fileCount int = 0
	uniqueCheck := make(map[string]int)
	for fn := range fileNames {
		file, err := os.Open(fn)
		if err != nil {
			return fileCount, err
		}
		defer file.Close()
		img, _, err := image.Decode(file)
		if err != nil {
			return fileCount, err
		}

		fileName := getFileName(fn)
		if _, ok := uniqueCheck[fileName]; !ok {
			uniqueCheck[fileName] = 0
		} else {
			uniqueCheck[fileName]++
			fileName = fileName + "(" + strconv.Itoa(uniqueCheck[fileName]) + ")"
		}

		dst, err := os.Create(fmt.Sprintf("output/%s.%s", fileName, to))
		if err != nil {
			return fileCount, err
		}
		defer dst.Close()

		switch to {
		case "jpeg", "jpg":
			err = jpeg.Encode(dst, img, nil)
		case "png":
			err = png.Encode(dst, img)
		}
		if err != nil {
			return fileCount, err
		}

		_, err = io.Copy(dst, file)
		if err != nil {
			return fileCount, err
		}
		fileCount++
	}
	return fileCount, nil
}

// globDir finds all the paths.
func globDir(dir string, ext string, fileNames chan<- string) {
	ue := strings.ToUpper(ext)
	for _, entry := range getDirEntry(dir) {
		if !strings.HasSuffix(entry.Name(), ext) &&
			!strings.HasSuffix(entry.Name(), ue) &&
			!entry.IsDir() {
			continue
		}
		if entry.IsDir() {
			globDir(filepath.Join(dir, entry.Name()), ext, fileNames)
		} else {
			fileNames <- filepath.Join(dir, entry.Name())
		}
	}
}

// getDirEntry returns an entry of directory.
func getDirEntry(dir string) []os.FileInfo {
	entries, err := ioutil.ReadDir(dir)
	if err != nil {
		return nil
	}
	return entries
}

// getFileName returns a file name from the full path.
func getFileName(path string) string {
	return filepath.Base(path[:len(path)-len(filepath.Ext(path))])
}

// validateArgs checks the image extension whether it is expected.
func validateArgs(from string, to string) error {
	ae := AllowedExt{"jpg", "jpeg", "png"}

	if to == from {
		err := errors.New("converter: from and to should be different")
		return err
	}
	if !ae.contains(to) {
		return fmt.Errorf("converter: from should be %s, your to is %s", ae, to)
	}
	if !ae.contains(from) {
		return fmt.Errorf("converter: from should be %s, your to is %s", ae, from)
	}

	return nil
}

type AllowedExt []string

// contains a method for AllowedExt to check whether the item is contained in its elements.
func (ae *AllowedExt) contains(item string) bool {
	set := make(map[string]struct{}, len(*ae))
	for _, s := range *ae {
		set[s] = struct{}{}
	}

	_, ok := set[item]
	return ok
}
