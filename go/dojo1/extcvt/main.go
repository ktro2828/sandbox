package main

import (
	"flag"
	"fmt"
	"io"
	"os"

	"github.com/ktro2828/converter"
)

type CLI struct {
	outStream,
	errStream io.Writer
}

func main() {
	cli := &CLI{outStream: os.Stdout, errStream: os.Stderr}
	os.Exit(cli.Run())
}

func (cli *CLI) Run() int {
	flag.Usage = usage
	flag.Parse()
	if len(os.Args[1:]) != 3 {
		usage()
		return 1
	}

	if err := os.MkdirAll("output", 0777); err != nil {
		fmt.Fprintln(cli.errStream, err)
		return 1
	}

	from := flag.Arg(0)
	to := flag.Arg(1)
	dir := flag.Arg(2)

	count, err := converter.ConvertExt(dir, from, to)
	if err != nil {
		fmt.Fprint(cli.errStream, err)
		return 1
	}

	fmt.Printf("%d files has been converted! See under ./output\n", count)

	return 0
}

func usage() {
	fmt.Println("Usage:")
	fmt.Println("  main extension(from) extension(to) targetdirectory")
	fmt.Println("")
	fmt.Println("All of the args are required.")
	flag.PrintDefaults()
}
