package main

import (
	"github.com/jacobsa/go-serial/serial"

	"bytes"
	"flag"
	"fmt"
	"io"
	"log"
	"net"
	"os"
	// "runtime/debug"
	"time"
)

var WIRE = flag.String("wire", "/dev/ttyACM0", "serial device connected by USB to Pi Pico")
var BAUD = flag.Uint("baud", 115200, "serial device baud rate")
var MCP = flag.String("mcp", "localhost:2321", "global LEMMA server or MCP server or local test MCP server")

var HANDLE = flag.String("handle", "ZZZ", "three-letter handle for this user.")
var NAME = flag.String("name", "No Name", "Your name or pseudonym.   If you are under 18, don't tell your name!")
var ZONE = flag.String("zone", "", "Override zone; default comes from --airport")
var AIRPORT = flag.String("airport", "ORD", "three-letter IANA airport destination code.")

var wire io.ReadWriteCloser
var mcp io.ReadWriteCloser
var channel chan byte

const (
	// Borrow $FFxx "addresses" for tagging Info
	HELLO_HANDLE  = 0xFF00
	HELLO_NAME    = 0xFF01
	HELLO_ZONE    = 0xFF02
	HELLO_AIRPORT = 0xFF03
)

func AxiomHostnameFromHandle() (z [8]byte) {
	h := *HANDLE
	n := len(h)
	for i := range z {
		if i < n {
			z[i] = h[i]
		} else {
			z[i] = ' '
		}
	}
	return
}

func sendAxiomStyleHellos() {
	// Enough to satisfy Lemma.

	pay_DF00 := make([]byte, 256)
	hostname := AxiomHostnameFromHandle()
	copy(pay_DF00[0xE0:0xE8], hostname[:])

	pay_01DA := make([]byte, 'T') // 64 + 'T' = 94 zeroes.

	// Now special tether fields for the MCP.
	// Finally the $0000 block, with the special handling greeting.
	const SpecialHandlingGreeting = "bonobo-nekotos"
	for _, rec := range []struct {
		id  uint
		val []byte
	}{
		{0xDF00, pay_DF00},
		{0x01DA, pay_01DA},
		{HELLO_HANDLE, []byte(*HANDLE)},
		{HELLO_NAME, []byte(*NAME)},
		{HELLO_ZONE, []byte(*ZONE)},
		{HELLO_AIRPORT, []byte(*AIRPORT)},
		{0, []byte(SpecialHandlingGreeting)}, // 0 must be last.
	} {
		Value(mcp.Write([]byte{1, Hi(len(rec.val)), Lo(len(rec.val)), Hi(rec.id), Lo(rec.id)}))
		Value(mcp.Write(rec.val))
	}
}

func logHex(prefix string, bb []byte) {
	if false {
		n := len(bb)
		for x := 0; x < n; x += 32 {
			var buf bytes.Buffer
			fmt.Fprintf(&buf, "%02x: ", x)
			for y := 0; x+y < n; y++ {
				fmt.Fprintf(&buf, "%02x ", bb[x+y])
				if (y & 3) == 3 {
					fmt.Fprintf(&buf, " ")
				}
			}
			for y := 0; x+y < n; y++ {
				c := bb[x+y]
				if 32 <= c && c <= 126 {
					fmt.Fprintf(&buf, "%c", bb[x+y])
				} else {
					fmt.Fprintf(&buf, "~")
				}
			}
			Logf("%q %s", prefix, buf.String())
		}
		Logf("")
	}
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
		if !ok {
			return
		}
		if x <= 128 {
			// Low numbers are logged to stderr.
			Value(os.Stderr.Write([]byte{x}))
		} else {
			// High number start a sequence to MCP.
			size := int(x - 128)
			for i := 0; i < size; i++ {
				y, ok := <-channel
				if !ok {
					return
				}
				bb[i] = y
			}
			Value(mcp.Write(bb[:size]))
			logHex("wire", bb[:size])
		}
	}
}

////////////////////////////////////////////////////////////////////////

func run() {
	// Open wire connection to Bonobo
	wire = Value(serial.Open(serial.OpenOptions{
		PortName:        *WIRE,
		BaudRate:        *BAUD,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 1,
	}))
	defer wire.Close()
	Logf("Serial wire opened %q", *WIRE)

	// TODO -- wait until Coco seems reset and Pico seems happy.
	// Open network connection to MCP
	mcp = Value(net.Dial("tcp", *MCP))
	defer mcp.Close()
	Logf("TCP mcp opened %q", *MCP)
	sendAxiomStyleHellos()

	channel = make(chan byte)

	go loopFromMcpToWire()
	go loopFromWireToChannel()
	loopFromChannelToMcp()
}

var previousRecover string

func tryRun() {
	// Make one attempt to run, catching any panic.
	defer func() {
		r := recover()
		if r != nil {
			s := Format("%v", r)
			if s != previousRecover {
				fmt.Printf("[recover: %q]\n", r)
				previousRecover = s
			}
		}
	}()
	Logf("tryRun...")
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

func assertEq[T Ordered](a, b T) {
	if a != b {
		log.Fatalf("assertEq fails: %v vs %v", a, b)
	}
}

type Ordered interface {
	~byte | ~int | ~uint | ~int64 | ~uint64 | ~rune | ~string
}
type Integer interface {
	~int | ~uint
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
		panic(s)
	}
}

var Logf = log.Printf
var Panicf = log.Panicf
var Format = fmt.Sprintf

func Hi[T Integer](x T) byte {
	return byte(x >> 8)
}
func Lo[T Integer](x T) byte {
	return byte(x >> 0)
}
