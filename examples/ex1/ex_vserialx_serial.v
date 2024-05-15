// Copyright (c) 2021 Erdet Nasufi, MIT License

module main

import time
import vserialx as vs

fn main() {
	dev_name := '/dev/ttyACM1'

	leds_on := [u8(0x01), 0x40, 0x47, 0x40, 0x47, 0x41, 0x41, 0x43, 0x44, 0x40, 0x7F, 0x7F, 0x05, 0x73, 0x0B, 0x03]
	leds_off := [u8(0x01), 0x40, 0x47, 0x40, 0x47, 0x41, 0x41, 0x43, 0x44, 0x40, 0x40, 0x40, 0x05, 0x64, 0xD9, 0x03]

	// mut dev := vs.new_default(dev_name)
	mut dev := vs.new(dev_name,
					.bps_9600, 
					.no_flow_control,
					.none_parity,
					.stop_bit_1,
					.char_size_8b)

	ret := dev.open()

	println('open(): ${ret}')
	if dev.connected() == false {
		println('Failed to open')
		return
	}

	// len, error := dev.write_string('Hello world.\n\n')
	// // len, error := dev.write([byte(0x31), 0x32, 0x33, 0x0A])
	// println('write_string: ${len} bytes, ${error}')
	// time.sleep(100000)
	
	for {
		n1, _ := dev.write(leds_on)
		if n1 == 0 {
			println("Failed to write.")
		}
		time.sleep( 1000*time.millisecond)
		_ := dev.flush()

		n2, _ := dev.write(leds_off)
		if n2 == 0 {
			println("Failed to write.")
		}
		time.sleep(1000*time.millisecond)
		_ := dev.flush()
	}

	dev.close()
}