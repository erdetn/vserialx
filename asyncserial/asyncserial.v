module asyncserial

#flag -I /usr/include/
#flag -I /usr/include/x86_64-linux-gnu/
#flag -I /usr/include/x86_64-linux-gnu/sys

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <features.h>

struct C.termios {
mut:
	c_iflag u32
    c_oflag u32
    c_cflag u32
    c_lflag u32
    c_line byte
    c_cc[32]byte
    c_ispeed u32
    c_ospeed u32
}

fn C.open(&char, u32) int
fn C.write(int, voidptr, usize) int
fn C.read(int, voidptr, usize) int
fn C.close(int)

fn C.tcgetattr(int, voidptr) int
fn C.tcsetattr(int, int, voidptr) int
fn C.tcsendbreak(int, int) int
fn C.tcdrain(int) int
fn C.tcflush(int, int) int
fn C.tcflow(int, int) int
fn C.cfmakeraw(voidptr)
fn C.cfgetispeed(voidptr) int
fn C.cfgetospeed(voidptr) int
fn C.cfsetispeed(voidptr, int)
fn C.cfsetospeed(voidptr, int) int
fn C.cfsetspeed(voidptr, int) int
fn C.ioctl(int, int, voidptr) int
fn C.isatty(int) int
fn C.fcntl(int, int, int) int

const (
	max_read_buffer  = 256
	max_write_buffer = 256
)

pub enum Baudrate {
	bps_0      = C.B0      // 0  [bps]
	bps_50     = C.B50     // 50 [bps]
	bps_75     = C.B75	   // 75 [bps]
	bps_110    = C.B110    // 110 [bps]
	bps_134    = C.B134    // 134.5 [bps]
	bps_150    = C.B150    // 150 [bps]
	bps_200    = C.B200    // 200 [bps]
	bps_300    = C.B300    // 300 [bps]
	bps_600    = C.B600    // 600 [bps]
	bps_1200   = C.B1200   // 1200 [bps]
	bps_1800   = C.B1800   // 1800 [bps]
	bps_2400   = C.B2400   // 2400 [bps]
	bps_4800   = C.B4800   // 4800 [bps]
	bps_9600   = C.B9600   // 9600 [bps]
	bps_19200  = C.B19200  // 19200 [bps]
	bps_38400  = C.B38400  // 38400 [bps]
	bps_57600  = C.B57600  // 57,600 [bps]
	bps_115200 = C.B115200 // 115,200 [bps]
}

const c_baudrates = [0 50 75 110 134 150 
					200	300 600 1200 1800 
					2400 4800 9600 7200 14400 
					19200 28800 38400]

pub enum AsyncSerialReturn {
	okay

	error_not_tty
	error_configuration_failed
	error_baudrate_mismatch
	error_port_not_open
	error_unknown
	error_bad_file_descriptor
	error_buffer_fault
	error_buffer_overlimit
	error_interrupted
	error_invalid_parameters
	error_io
	error_no_space
	error_no_permission
	error_closed_pipe
	error_no_address
}

pub enum Parity {
	none_parity
	odd_parity
	even_parity
}

pub enum FlowControl {
	hardware_flow_control
	software_flow_control
	no_flow_control
}

pub enum StopBit {
	stop_bit_1
	stop_bit_2
}

pub enum CharacterSize {
	char_size_5b
	char_size_6b
	char_size_7b
	char_size_8b
}

pub struct AsyncSerial {
mut:
	fd             int
	port_name      string
	baud_rate      Baudrate
	is_connected   bool
	io_blocking    bool
	flow_control   FlowControl
	parity         Parity
	stop_bit       StopBit
	char_size      CharacterSize

	lock_port      bool = true

	flag_ignore_cr bool = true
	flag_map_cr_nl bool
	flag_map_nl_cr bool

	timeout        u32  = 100 // [ms]
}

pub fn new_default(port_name string) AsyncSerial {
	this := AsyncSerial {
		fd:             -1
		baud_rate:     .bps_9600
		flow_control:  .no_flow_control
		parity:        .none_parity
		stop_bit:      .stop_bit_1
		char_size:     .char_size_8b
		port_name:     port_name
	}

	return this
}

pub fn new(port_name string, baud_rate Baudrate, flow_control FlowControl,
		   parity Parity, stop_bit StopBit, char_size CharacterSize) AsyncSerial {
	this := AsyncSerial {
		fd:             -1
		baud_rate:     baud_rate
		flow_control:  flow_control
		parity:        parity
		stop_bit:      stop_bit
		char_size:     char_size
		port_name:     port_name
	}

	return this
}

pub fn (mut this AsyncSerial)open() AsyncSerialReturn {

	mut open_flag := u32(C.O_RDWR | C.O_NOCTTY)
	if this.io_blocking == false {
		open_flag |= u32(C.O_NDELAY)
	} else {
		open_flag &= u32(~(C.O_NDELAY))
	}

	this.fd = int(C.open(this.port_name.str, open_flag))
	if this.fd == -1 {
		return this.get_error(this.fd)
	}
	this.is_connected = true

	mut rc := C.isatty(this.fd)
	if rc != 1 {
		vlogd('Not a tty device.')
		this.close()
		return AsyncSerialReturn.error_not_tty
	}

	if this.lock_port == true {
		C.ioctl(this.fd, C.TIOCEXCL,  0)
	} else {
		C.ioctl(this.fd, C.TIOCGEXCL, 0)
	}
 
	rc = int(this.flush())
	if rc != int(AsyncSerialReturn.okay) {
		vlogd('Failed to flush (${rc}).')
	}

	mut options := C.termios{}
	rc = int(C.tcgetattr(this.fd, &options))
  	if rc != 0 {
    	vlogd("Failed to get settings (${rc}).")
    	this.close()
		return AsyncSerialReturn.error_unknown
    }

	options.c_iflag |= u32((C.INPCK | C.ISTRIP))
	if this.flag_ignore_cr == true {
		options.c_iflag &= u32(~(C.IGNCR))
	} else {
		options.c_iflag |= u32((C.IGNCR))
	}

	if this.flag_map_nl_cr == false {
		options.c_iflag &= u32(~(C.INLCR))
	} else {
		options.c_iflag |= u32(C.INLCR)
	}

	if this.flag_map_cr_nl == false {
		options.c_iflag &= u32(~(C.ICRNL))
	} else {
		options.c_iflag |= u32(C.ICRNL)
	}

	options.c_oflag &= u32(~(C.ONLCR | C.OCRNL | C.OPOST))
	options.c_lflag &= u32(~(C.ECHO | C.ECHONL | C.ICANON | C.ISIG | C.IEXTEN))

	// Set up timeouts: Calls to read() will return as soon as there is
	// at least one byte available or when 100 ms has passed.
	options.c_cc[C.VTIME] = byte((this.timeout)/100)
	options.c_cc[C.VMIN] = 0
 
	// Set baudrate
	C.cfsetospeed(&options, int(this.baud_rate))
	C.cfsetispeed(&options, int(this.baud_rate))
	if int(C.cfgetospeed(&options)) != int(this.baud_rate) {
		vlogd('Baud rate is not set.')
		return AsyncSerialReturn.error_baudrate_mismatch
	}

	// set CFLAG
	options.c_cflag |= u32((C.CLOCAL | C.CREAD))

	// Configure flow control 
	match this.flow_control {
		.hardware_flow_control{
			options.c_cflag |= u32(C.CRTSCTS)
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
		}
		.software_flow_control{
			options.c_iflag |= u32((C.IXON | C.IXOFF | C.IXANY))
			options.c_iflag |= u32((C.IXON | C.IXOFF | C.IXANY))
			options.c_cflag &= u32(~(C.CRTSCTS))
		}
		else{
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
			options.c_cflag &= u32(~(C.CRTSCTS))
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
		}
	}

	// Configure parity
	options.c_iflag |= u32((C.INPCK | C.ISTRIP))
	match this.parity {
		.none_parity {
			options.c_cflag &= u32(~(C.PARENB))
		}
		.even_parity {
			options.c_cflag |= u32((C.PARENB))
			options.c_cflag &= u32(~(C.PARODD))
		}
		.odd_parity {
			options.c_cflag |= u32(C.PARENB)
			options.c_cflag |= u32(C.PARODD)
		}
	}

	// Configure stop bit
	match this.stop_bit {
		.stop_bit_1 {
			options.c_cflag &= u32(~(C.CSTOPB))
		}
		.stop_bit_2 {
			options.c_cflag |= u32(C.CSTOPB)
		}
	}

	// Configure character size
	options.c_cflag &= u32(~(C.CSIZE))
	options.c_cflag &= u32(~(C.CS5 | C.CS6 | C.CS7 | C.CS8))
	match this.char_size {
		.char_size_5b {
			options.c_cflag |= u32(C.CS5)
		}
		.char_size_6b {
			options.c_cflag |= u32(C.CS6)
		}
		.char_size_7b {
			options.c_cflag |= u32(C.CS7)
		}
		.char_size_8b {
			options.c_cflag |= u32(C.CS8)
		}
	}

	// Set final configuration
	rc = int(C.tcsetattr(this.fd, C.TCSANOW, &options))
	if rc != 0 {
		vlogd('Failed to set final configuration (${rc})')
		this.close()
		return AsyncSerialReturn.error_configuration_failed
	}
	
	return AsyncSerialReturn.okay
}

pub fn (mut this AsyncSerial)close() {
	if this.is_connected == false {
		return
	}

	this.is_connected = false
	C.close(this.fd)
}

pub fn (mut this AsyncSerial) connected() bool {
	return this.is_connected
}

pub fn (mut this AsyncSerial)has_data() u32 {
	mut bytes_available := u32(0)

	rc := C.ioctl(this.fd, C.FIONREAD, &bytes_available)
	if rc != 0 {
		return 0
	}
	return u32(bytes_available)
}

fn (mut this AsyncSerial)read_ex(maxbytes int) (int, []byte, AsyncSerialReturn){
	unsafe {
		mut buf := malloc_noscan(maxbytes + 1)
		nbytes := C.read(this.fd, buf, maxbytes)
		if nbytes < 0 {
			free(buf)
			return 1, [byte(0)], this.get_error(nbytes)
		}
		buf[nbytes] = 0
		return nbytes, buf.vbytes(nbytes), AsyncSerialReturn.okay
	}
}

pub fn (mut this AsyncSerial)read(maxbytes int)(int, []byte, AsyncSerialReturn){
	C.fcntl(this.fd, C.F_SETFL, C.FNDELAY)

	return this.read_ex(maxbytes)
}

pub fn (mut this AsyncSerial)wait_reading(maxbytes int) (int, []byte, AsyncSerialReturn){
	C.fcntl(this.fd, C.F_SETFL, 0)
	return this.read_ex(maxbytes)
}

pub fn (mut this AsyncSerial)write(package string) (u32, AsyncSerialReturn){
	rc := int(C.write(this.fd, package.str, usize(package.len)))
	
	if rc > 0 {
		return u32(rc), AsyncSerialReturn.okay
	}

	error := this.get_error(rc)
	return 0, error
}

pub fn (mut this AsyncSerial)write_raw(package []byte) (u32, AsyncSerialReturn){
	rc := int(C.write(this.fd, voidptr(&package[0]), usize(package.len)))
	
	if rc > 0 {
		return u32(rc), AsyncSerialReturn.okay
	}

	error := this.get_error(rc)
	return 0, error
}

pub fn (mut this AsyncSerial)flush_read() AsyncSerialReturn{
	rc := C.tcflush(this.fd, C.TCIFLUSH)
	return this.get_error(rc)
}

pub fn (mut this AsyncSerial)flush_write() AsyncSerialReturn {
	rc := C.tcflush(this.fd, C.TCOFLUSH)
	return this.get_error(rc)
}

pub fn (mut this AsyncSerial)flush() AsyncSerialReturn{
	rc := C.tcflush(this.fd, C.TCIOFLUSH)
	return this.get_error(rc)
}

pub fn (mut this AsyncSerial)get_error(error_code int) AsyncSerialReturn {
	return match  error_code {
		C.EBADF {
			AsyncSerialReturn.error_bad_file_descriptor
		}
		C.EDESTADDRREQ {
			AsyncSerialReturn.error_no_address
		}
		C.EFAULT {
			AsyncSerialReturn.error_buffer_fault
		}
		C.EFBIG {
			AsyncSerialReturn.error_buffer_overlimit
		}
		C.EINTR{
			AsyncSerialReturn.error_interrupted
		}
		C.EINVAL{
			AsyncSerialReturn.error_invalid_parameters
		}
		C.EIO {
			AsyncSerialReturn.error_io
		}
		C.ENOSPC {
			AsyncSerialReturn.error_no_space
		}
		C.EPERM{
			AsyncSerialReturn.error_no_permission
		}
		C.EPIPE {
			AsyncSerialReturn.error_closed_pipe
		}
		else {
			AsyncSerialReturn.error_unknown
		}
	}
}

pub fn (mut this AsyncSerial)lock_access() {
	this.lock_port = true
	C.ioctl(this.fd, C.TIOCEXCL,  0)
}

pub fn (mut this AsyncSerial)unlock_access() {
	this.lock_port = false
	C.ioctl(this.fd, C.TIOCGEXCL,  0)
}

pub fn (mut this AsyncSerial)is_locked() bool {
	return this.lock_port
}

pub fn (mut this AsyncSerial)get_baudrate() ?int {
	mut tx_baudrate := int(0)
	mut rx_baudrate := int(0)
	mut options := C.termios{}


	rc := int(C.tcgetattr(this.fd, &options))
  	if rc != 0 {
		return error('Failed to get options.')
    }

	tx_baudrate = C.cfgetospeed(&options)
	rx_baudrate = C.cfgetispeed(&options)

	if tx_baudrate != rx_baudrate {
		return error('Baudrate mismatch.')
	}

	if tx_baudrate > 13 || rx_baudrate > 13 {
		return error('Unknown baudrate.')
	}

	return c_baudrates[tx_baudrate]
}

[inline]
fn vlogd(message string) {
	$if debug {
		println(message)
	}
}
