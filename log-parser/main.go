package main

import (
	"bufio"
	// "encoding/binary"

	"encoding/hex"
	"fmt"
	"strings"

	"os"
)

func main() {
	if len(os.Args) < 2 {
		panic("No crd log provided")
	}
	crdLogFilename := os.Args[1]
	file, err := os.Open(crdLogFilename)
	if err != nil {
		panic(err)
	}

	_, err = file.Stat()
	if err != nil {
		panic(err)
	}

	fmt.Println(
		"time,inletTemp,outletTemp,flowRate,pressure,compressorPWM,shirtPWM,fault,",
		"systemEnable,chillerPumpEnabled,resetRequired,coolantLevel,undertempCutoff,systemStatus",
	)
	defer file.Close()
	r := bufio.NewReader(file)
	for {
		line, _, err := r.ReadLine()
		if err != nil {
			break
		}

		lineStr := string(line)
		parts := strings.Split(lineStr, " ")
		if len(parts) != 11 {
			// skipping comment line
			continue
		}

		pBytes, err := hex.DecodeString(strings.Join(parts[3:11], ""))
		if err != nil {
			panic(err)
		}
		inletTemp := (float64(uint8(pBytes[0])) * 0.25) - 15
		outletTemp := (float64(uint8(pBytes[1])) * 0.25) - 15
		flowRate := uint(uint8(pBytes[2])) * 15
		systemPressure := uint(uint8(pBytes[3])) * 4
		compressorPWM := uint8(pBytes[4])
		shirtPWM := uint8(pBytes[5])

		// 6    | flags bitmap
		//      |   [7]: system enable
		//      |   [6]: chiller pump active
		//      |   [5]: system reset required
		//      |   [4]: coolant level OK
		//      |   [3]: under-temp compressor cut-off
		//      |   [2,1,0]: system status
		flags := uint8(pBytes[6])
		systemEnable := (flags & 0x80) >> 7
		chillerPumpEnabled := (flags & 0x40) >> 6
		resetRequired := (flags & 0x20) >> 5
		coolantLevel := (flags & 0x10) >> 4
		undertempCutoff := (flags & 0x8) >> 3
		systemStatus := (flags & 0x7)

		fmt.Printf(
			"%s,%0.2f,%0.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			parts[0],
			inletTemp,
			outletTemp,
			flowRate,
			systemPressure,
			compressorPWM,
			shirtPWM,
			uint8(pBytes[7]),
			systemEnable,
			chillerPumpEnabled,
			resetRequired,
			coolantLevel,
			undertempCutoff,
			systemStatus,
		)
	}
}
