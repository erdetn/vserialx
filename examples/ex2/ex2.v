// Copyright (c) 2021 Erdet Nasufi, MIT License

module main

import vserialx

fn main() {
	tty_port_name := '/dev/ttyS0'

	mut tty_port := vserialx.new(tty_port_name,
					.b9600, 
					.@none,
					.@none,
					.one,
					.char8b)

	ret := tty_port.open()

	println('open(): ${ret}')
	if tty_port.is_connected() == false {
		println('Failed to open')
		return
	}

	baudrate := tty_port.baudrate() or {
		-1
	}
	
	println('Baudrate: ${baudrate}')

	tty_port.close()
}