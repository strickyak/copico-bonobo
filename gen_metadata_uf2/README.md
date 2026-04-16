# RP2350 Metadata Injector

(* code by Google Gemini 3 *)

This tool generates a **UF2** (USB Flashing Format) file containing a 256-byte metadata payload (such as a serial number, hardware revision, or build date). It is designed specifically to write this data to the last page of a **16MB (128Mbit)** QSPI Flash chip on an RP2350 board.

## How It Works

The RP2350 series features an **XIP (Execute In Place)** controller that maps external QSPI Flash directly into the processor's memory space. For a 16MB chip like the W25Q128, this memory is mapped from `0x10000000` to `0x10FFFFFF`.

This Go tool creates a single-block UF2 file that writes your custom ASCII string to the very last 256-byte page of that memory space: **`0x10FFFF00`**. The remainder of the 256-byte block is automatically zero-padded.

## Prerequisites

* [Go](https://go.dev/) installed on your host machine to build the generator.
* An RP2350B board with 16MB of Flash (using the ARM Cortex-M33 architecture).

## Usage

### 1. Build the Tool

Compile the Go program into an executable:

```bash
go build -o gen_metadata_uf2 main.go
```

### 2. Generate the UF2 File

Run the tool, providing your desired metadata string via the --label flag and the output filename via --uf2:

```bash
./gen_metadata_uf2 --label "SN-2026-BOARD-REV-B" --uf2 metadata.uf2
```

(Note: The --label payload must be between 0 and 255 bytes).

### 3. Deploy to the RP2350

1.  Put your RP2350 board into BOOTSEL mode (hold the BOOTSEL button while plugging it into USB, or resetting it).

2.  A mass storage drive named RP2-UE2 will appear.

3.  Drag and drop metadata.uf2 onto the drive. The board will briefly flash and reboot.

## Reading the Data in C++ (Pico SDK)

Because the data is mapped directly into the XIP address space, you do
not need to write complex SPI routines to read it. You can access it
directly via a memory pointer in your firmware.

```C++
#include <stdio.h>
#include "pico/stdlib.h"

// The address of the last 256-byte page on a 16MB Flash chip
#define METADATA_ADDR (const char *)(0x10FFFF00)

void print_metadata() {
    // Treat the memory address as a standard C-string
    // The Go tool zero-pads the payload, ensuring null-termination
    printf("Hardware Metadata: %s\n", METADATA_ADDR);
}

int main() {
    stdio_init_all();

    // Wait for USB connection to see the output (optional)
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    print_metadata();

    while (true) {
        tight_loop_contents();
    }
}
```

## Important Caveats

### 1. The Sector Erase Side Effect

Flash memory cannot overwrite individual bytes; it must be erased in 4KB
sectors (0x1000 bytes). Even though this UF2 file only writes a 256-byte
payload, the RP2350 bootloader must erase the entire final 4KB sector
(0x10FFF000 to 0x10FFFFFF) before writing.

Danger: If your main application binary is larger than 16,773,120 bytes
(~16.38 MB), it will overlap into this final sector. Flashing this
metadata will corrupt the end of your program.

### 2. Architecture Specificity

This tool hardcodes the UF2 Family ID to 0xe48bff56, which is specific to
the RP2350 ARM architecture. If you compile your firmware to run on the
Hazard3 RISC-V cores, you must modify the Go source code to use the RISC-V
Family ID (0xabcc8211), otherwise the bootloader may reject the file.

### 3. Flash Wipes

If you use a "nuke" UF2 (like flash_nuke.uf2) or perform a full chip erase
via a debug probe (picoprobe/SWD), this metadata will be permanently
destroyed and will need to be re-flashed. Normal firmware updates
(flashing a standard application UF2) will leave this metadata intact,
provided the application isn't large enough to overwrite the final
4KB sector.
