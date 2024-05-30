// Copyright (c) 2021 Erdet Nasufi, MIT License

module main

import time
import vserialx as vs

fn main() {
	dev_name := '/dev/ttyS0'

	// mut dev := vs.new_default(dev_name)
	mut dev := vs.new(dev_name,
					.b9600, 
					.@none,
					.@none,
					.one,
					.char8b)

	ret := dev.open()

	println('open(): ${ret}')
	if dev.is_connected() == false {
		println('Failed to open.\nERROR: ${ret}')
		return
	}

	// len, error := dev.write_string('Hello world.\n\n')

	len, rc := dev.write([byte(0x31), 0x32, 0x33, 0x0A])
	println('write_string: ${len} bytes, ${rc}')
	time.sleep(time.second)

	if rc == .okay {
		println('Sent.')
	} else {
		println('ERROR: failed to send buffer.\nCode: ${rc}')
	}

	dev.close()
}