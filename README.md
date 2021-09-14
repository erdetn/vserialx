# vasyncserial
Tiny wrapper for serial communication in V.
Tested only on `Linux ex270 5.11.0-27-generic #29~20.04.1-Ubuntu x86_64 x86_64 x86_64 GNU/Linux`.

## Usage

```
module main

import time
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

	len, error := tty_port.write('Hello world.\n\n')
	//len, error := tty_port.write_raw([byte(0x31), 0x32, 0x33, 0x0A])
	println('write_raw: ${len} bytes, ${error}')
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
- Exclusive access (locking and unlocking mechanism).
- Reading configurations using `tcgetattr` and `ioctl`.
[+] Better error/return code naming
- The documentation

## TODO testings
- Other serial port configurations need to be tested.
- Bloking read/write need to be tested.