# vserialx
**vserialx** is a tiny library for serial communication in Linux using V.

## Usage

```v
module main

import time
import vserialx

fn main() {
	tty_port_name := '/dev/ttyUSB0'

	// mut tty_port := asyncserial.new_default(tty_port_name)
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

	// send string message
	len, error := tty_port.write_string('Hello world.\n\n')
	time.sleep(10000)

	// send raw byte array
	len, error := tty_port.write([byte(0x31), 0x32, 0x33, 0x0A])
	println('write_buffer: ${len} bytes, ${error}')
	time.sleep(100000)
	
	for{
		rx_bytes := tty_port.has_data()
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
```

## TODO features

- [ ] Exclusive access (locking and unlocking mechanism).
- [ ] Reading configurations using `tcgetattr` and `ioctl`. Read only the baudrate for now.
- [ ] Better error/return code naming
- [ ] The documentation

## TODO testings
- [ ] Other serial port configurations need to be tested.
- [ ] Bloking read/write need to be tested.