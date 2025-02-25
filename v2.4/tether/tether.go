package main

import (
	"github.com/jacobsa/go-serial/serial"

	//"bytes"
	"flag"
	"fmt"
	"io"
	"log"
    "net"
	//"os"
	//"os/exec"
	//"os/signal"
	//"regexp"
	//"strconv"
	//"strings"
	//"syscall"
	"time"
    "runtime/debug"
)

var WIRE = flag.String("wire", "/dev/ttyACM0", "serial device connected by USB to Pi Pico")
var BAUD = flag.Uint("baud", 115200, "serial device baud rate")
var FIRMWARE = flag.String("loadm", "nekot-bonobo", "binary to load on startup")
var MCP = flag.String("mcp", "localhost:14511", "global MCP server or local test MCP server")
var HANDLE = flag.String("handle", "ZZZ", "three-letter handle for this user")

var fromWire chan packet
var toWire chan packet
var wire io.ReadWriteCloser

var fromMcp chan packet
var toMcp chan packet
var mcp io.ReadWriteCloser

type word uint16

type packet struct {
    c byte
    p word
    v []byte
}

func loopFromWire() {
    for {
        var bb [5]byte
        read(wire, bb[:])
        c := bb[0]
        n := hilo(bb[1], bb[2])
        p := hilo(bb[3], bb[4])
        v := make([]byte, n)
        read(wire, v)
        fromWire <- packet{c, p, v}
    }
}

func loopToWire() {
    for {
        a := <- toWire
        n := word(len(a.v))
        q := [5]byte{a.c, hi(n), lo(n), hi(a.p), lo(a.p)}
        write(wire, q[:])
        write(wire, a.v)
    }
}

func loopFromMcp() {
    for {
        var bb [5]byte
        read(wire, bb[:])
        c := bb[0]
        n := hilo(bb[1], bb[2])
        p := hilo(bb[3], bb[4])
        v := make([]byte, n)
        read(wire, v)
        fromMcp <- packet{c, p, v}
    }
}

func loopToMcp() {
    for {
        a := <- toMcp
        n := word(len(a.v))
        q := [5]byte{a.c, hi(n), lo(n), hi(a.p), lo(a.p)}
        write(wire, q[:])
        write(wire, a.v)
    }
}

func run() {
    // Open wire connection to Bonobo
	options := serial.OpenOptions{
		PortName:        *WIRE,
		BaudRate:        *BAUD,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 1,
	}
	wire = Value(serial.Open(options))
	defer wire.Close()
    Log("Serial wire opened %q", *WIRE)

    // Open network connection to MCP
    mcp = Value(net.Dial("tcp", *MCP))
    defer mcp.Close()
    Log("TCP mcp opened %q", *MCP)

    go loopToWire()
    go loopToMcp()

    for {
        select {
            case a := <- fromWire:
                doFromWire(a)
            case a := <- fromMcp:
                doFromMcp(a)
        }
    }
}

func tryRun() {
	defer func() {
		r := recover()
		if r != nil {
			fmt.Printf("[recover: %q]\n", r)
		}
	}()
	run()
}

func main() {
	log.SetFlags(0)
	log.SetPrefix("# ")
	flag.Parse()

    for {
        tryRun()
        time.Sleep(time.Second)
    }
}

const (
    LOG_PACKET = 7
)

func doFromWire(a packet) {
    if a.c == LOG_PACKET {
        Log("fromWire: %q", a.v)
        return
    }

    toMcp <- a
}

func doFromMcp(a packet) {
    if a.c == LOG_PACKET {
        Log("fromMcp: %q", a.v)
        return
    }

    toWire <- a
}

func read(r io.Reader, bb []byte) {
        n := Value(io.ReadFull(r, bb))
        assertEq(n, len(bb))
}
func write(w io.Writer, bb []byte) {
        n := Value(w.Write(bb))
        assertEq(n, len(bb))
}

func hi(a word) byte {
    return byte(a>>8)
}
func lo(a word) byte {
    return byte(a)
}
func hilo(hi, lo byte) word {
    return (word(hi)<<8) | word(lo)
}

func assertEq[T Ordered](a, b T) {
    if a != b {
        log.Fatalf("assertEq fails: %v vs %v", a, b)
    }
}
type Ordered interface {
    ~byte | ~int | ~uint | ~int64 | ~uint64 | ~rune | ~string
}

func Value[T any](value T, err error) T {
        Check(err)
        return value
}

func Check(err error, args ...any) {
        if err != nil {
                s := fmt.Sprintf("Check Fails: %v", err)
                for _, x := range args {
                        s += fmt.Sprintf(" ; %v", x)
                }
                s += "\n[[[[[[\n" + string(debug.Stack()) + "\n]]]]]]\n"
                log.Panic(s)
        }
}

var Log = log.Printf
var Panicf = log.Panicf
