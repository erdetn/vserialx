// Copyright (c) 2024 Erdet Nasufi, MIT License

module main

import time
import vserialx as vs

fn main() {
	mut dev := vs.new('/dev/ttyS20', .b9600, .@none, .@none, .one, .char8b)

	mut rc := dev.open()

	if rc != .okay {
		println('Failed to open.\nERROR: ${rc}')
		return
	}
	defer {
		dev.close()
	}

	rc = dev.open()
	if rc != .okay {
		println('Failed to connect.\nERROR: ${rc}')
		return
	}

	for {
		if dev.count() > 0 {
			len, buff, rt := dev.read(10)
			dump(rt)
			if len > 0 {
				println('BUFF: {${buff.hex()}}')
			}
		}
		time.sleep(time.second)
	}
}
