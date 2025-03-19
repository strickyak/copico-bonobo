package main

import (
	"github.com/jacobsa/go-serial/serial"

	"bytes"
	"flag"
	"fmt"
	"io"
	"os"
	"log"
	"net"
	"runtime/debug"
	"time"
)

var WIRE = flag.String("wire", "/dev/ttyACM0", "serial device connected by USB to Pi Pico")
var BAUD = flag.Uint("baud", 115200, "serial device baud rate")
var FIRMWARE = flag.String("firmware", "nekot-bonobo", "binary to load on startup")
var MCP = flag.String("mcp", "localhost:2321", "global LEMMA server or MCP server or local test MCP server")
var HANDLE = flag.String("handle", "ZZZ", "three-letter handle for this user")

var wire io.ReadWriteCloser
var mcp io.ReadWriteCloser
var channel chan byte

func helloMcp() {
    // Enough to satisfy the MCP.
    q1 := []byte{1, 1, 0, 0xDF, 0x00}
    p1 := make([]byte, 256)
    copy(p1[0xE0:0xE8] , []byte("SFO     "))

    q2 := []byte{1, 0, 'T', 0x01, 0xDA}
    p2 := make([]byte, 'T')

    const Greeting = "bonobo-nekot1"
    q3 := []byte{1, 0, byte(len(Greeting)), 0, 0}
    p3 := []byte(Greeting)

    for _, bb := range [][]byte{q1, p1, q2, p2, q3, p3} {
        Value(mcp.Write(bb))
    }
}

func logHex(prefix string, bb []byte) {
    n := len(bb)
    for x := 0; x < n; x += 32 {
        var buf bytes.Buffer
        fmt.Fprintf(&buf, "%02x: ", x)
        for y := 0; x + y < n; y++ {
            fmt.Fprintf(&buf, "%02x ", bb[x+y])
            if (y&3)==3 {
                fmt.Fprintf(&buf, " ")
            }
        }
        for y := 0; x + y < n; y++ {
            c := bb[x+y]
            if 32 <= c && c <= 126 {
                fmt.Fprintf(&buf, "%c", bb[x+y])
            } else {
                fmt.Fprintf(&buf, "~")
            }
        }
        Logf("%q %s", prefix, buf.String());
    }
    Logf("")
}

func loopFromMcpToWire() {
    const N = 4096
	var bb [N]byte
	for {
        count := Value(mcp.Read(bb[:]))
        Value(wire.Write(bb[:count]))
        logHex("mcp", bb[:count])
	}
}

func loopFromWireToChannel() {
    defer func() {
        r := recover()
        if r != nil {
            Logf("loopFromWireToChannel: Closing channel because %v", r)
            close(channel)
        }
    }()

    const N = 128
	var bb [N]byte
    for {
        count := Value(wire.Read(bb[:]))
        for i := 0; i < count; i++ {
            channel <- bb[i]
        }
    }
}

func loopFromChannelToMcp() {
    defer func() {
        r := recover()
        if r != nil {
            Logf("loopFromChannelToMcp: Closing mcp because %v", r)
            mcp.Close()
        }
    }()

    const N = 128
	var bb [N]byte
    for {
        x, ok := <-channel
        if (!ok) {
            return
        }
        if x <= 128 {
            // Low numbers are logged to stderr.
            Value(os.Stderr.Write([]byte{x}))
        } else {
            // High number start a sequence to MCP.
            size := int(x-128)
            for i:=0; i < size ; i++ {
                y, ok := <-channel
                if (!ok) {
                    return
                }
                bb[i] = y
            }
            Value(mcp.Write(bb[:size]))
            logHex("wire", bb[:size])
        }
    }
}

func feedFirmwareToWire() {
    for i := 10; i > 0; i-- {
        Logf("delay %d");
        time.Sleep(time.Second)
    }
    Logf("Go.")
    bb := Value(os.ReadFile(*FIRMWARE))
    Value(wire.Write(bb))
}

////////////////////////////////////////////////////////////////////////

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
	Logf("Serial wire opened %q", *WIRE)

	// Open network connection to MCP
	mcp = Value(net.Dial("tcp", *MCP))
	defer mcp.Close()
	Logf("TCP mcp opened %q", *MCP)
    helloMcp()

    channel = make(chan byte)

/*
    feedFirmwareToWire();
*/

	go loopFromMcpToWire()
	go loopFromWireToChannel()
    loopFromChannelToMcp()
}

func tryRun() {
    // Make one attempt to run, catching any panic.
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
	log.SetPrefix("T# ")
	flag.Parse()

	for {
		tryRun()
        // when disconnected, wait a second before retry.
		time.Sleep(time.Second)
	}
}

/*
func hi(a word) byte {
	return byte(a >> 8)
}
func lo(a word) byte {
	return byte(a)
}
func hilo(hi, lo byte) word {
	return (word(hi) << 8) | word(lo)
}
*/

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

var Logf = log.Printf
var Panicf = log.Panicf
