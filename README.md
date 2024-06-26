# vserialx
**vserialx** is a tiny library for serial communication in Linux using V.

## Usage

```v
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
		if dev.available_bytes() > 0 {
			len, buff, rt := dev.read(10)
			dump(rt)
			if len > 0 {
				println('BUFF: {${buff.hex()}}')
			}
		}
		time.sleep(time.second)
	}
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