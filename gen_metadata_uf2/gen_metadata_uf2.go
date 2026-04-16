package main

// code by Google Gemini 3

import (
	"encoding/binary"
	"flag"
	"fmt"
	"log"
	"os"
)

const (
	UF2_MAGIC_START0 = 0x0A324655
	UF2_MAGIC_START1 = 0x9E5D5157
	UF2_MAGIC_END    = 0x0AB16F30

	// RP2350 ARM Family ID
	RP2350_ARM_FAMILY_ID = 0xe48bff56

	// UF2 Flags: 0x00002000 means FamilyID is present
	UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000

	// Target address for the last page of a 16MB Flash
	TARGET_ADDR = 0x10FFFF00

	BLOCK_SIZE   = 512
	PAYLOAD_SIZE = 256
)

func main() {
	labelPtr := flag.String("label", "", "ASCII string to store (0-255 bytes)")
	uf2PathPtr := flag.String("uf2", "_metadata.uf2", "Output filename")
	flag.Parse()

	if len(*labelPtr) > PAYLOAD_SIZE {
		log.Fatalf("Error: Label is %d bytes, max is %d", len(*labelPtr), PAYLOAD_SIZE)
	}

	// Create the 512-byte buffer for a single UF2 block
	block := make([]byte, BLOCK_SIZE)

	// 1. Start Magics
	binary.LittleEndian.PutUint32(block[0:4], UF2_MAGIC_START0)
	binary.LittleEndian.PutUint32(block[4:8], UF2_MAGIC_START1)

	// 2. Flags
	binary.LittleEndian.PutUint32(block[8:12], UF2_FLAG_FAMILY_ID_PRESENT)

	// 3. Target Address
	binary.LittleEndian.PutUint32(block[12:16], TARGET_ADDR)

	// 4. Payload Size (Fixed at 256 for Pico)
	binary.LittleEndian.PutUint32(block[16:20], PAYLOAD_SIZE)

	// 5. Block Number (0 for a single-block file)
	binary.LittleEndian.PutUint32(block[20:24], 0)

	// 6. Number of Blocks (1 total)
	binary.LittleEndian.PutUint32(block[24:28], 1)

	// 7. Family ID (RP2350 ARM)
	binary.LittleEndian.PutUint32(block[28:32], RP2350_ARM_FAMILY_ID)

	// 8. Data Payload (Starts at offset 32)
	copy(block[32:32+len(*labelPtr)], *labelPtr)
	// Remaining bytes of the 256-byte payload are already 0 from make()

	// 9. End Magic (Starts at the very end of the 512-byte block)
	binary.LittleEndian.PutUint32(block[508:512], UF2_MAGIC_END)

	// Write to file
	err := os.WriteFile(*uf2PathPtr, block, 0644)
	if err != nil {
		log.Fatalf("Failed to write file: %v", err)
	}

	fmt.Printf("Successfully generated %s\n", *uf2PathPtr)
	fmt.Printf("Target Address: 0x%08X\n", TARGET_ADDR)
	fmt.Printf("Label: \"%s\"\n", *labelPtr)
}
