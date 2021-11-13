// Copyright (c) 2021 Erdet Nasufi, MIT License

module main

import time
import vserialx

fn main() {
	tty_port_name := '/dev/ttyUSB0'

	// mut tty_port := vserialx.new_default(tty_port_name)
	mut tty_port := vserialx.new(tty_port_name,
					.bps_9600, 
					.no_flow_control,
					.none_parity,
					.stop_bit_1,
					.char_size_8b)

	ret := tty_port.open()

	println('open(): ${ret}')
	if tty_port.connected() == false {
		println('Failed to open')
		return
	}

	len, error := tty_port.write_string('Hello world.\n\n')
	// len, error := tty_port.write([byte(0x31), 0x32, 0x33, 0x0A])
	println('write_string: ${len} bytes, ${error}')
	time.sleep(100000)
	
	for {
		rx_bytes := tty_port.available_bytes()
		time.sleep(10000)
		if rx_bytes > 0 {
			nbytes, buff, _ := tty_port.read(100)
			dump(buff)
			dump(nbytes)
		}
		time.sleep(100000)
	}

	tty_port.close()
}