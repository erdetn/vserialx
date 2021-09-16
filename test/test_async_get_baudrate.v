module main

import asyncserial

fn main() {
	tty_port_name := '/dev/ttyUSB0'

	// mut tty_port := asyncserial.new_default(tty_port_name)
	mut tty_port := asyncserial.new(tty_port_name,
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

	baudrate := tty_port.get_baudrate() or {
		-1
	}

	println('Baudrate: ${baudrate}')

	tty_port.close()
}