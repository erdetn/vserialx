// Copyright (c) 2021 Erdet Nasufi, MIT License

module vserialx

$if !linux {
    $compile_error('Only Linux is supported.')
}

#include "fcntl.h"
#include "stdio.h"
#include "unistd.h"
#include "stdint.h"
#include "sys/ioctl.h"
#include "termios.h"
#include "errno.h"
#include "features.h"

struct C.termios {
mut:
	c_iflag  u32
	c_oflag  u32
	c_cflag  u32
	c_lflag  u32
	c_line   u8
	c_cc     [32]u8
	c_ispeed u32
	c_ospeed u32
}

fn C.open(&char, u32) int
fn C.write(int, voidptr, usize) int
fn C.read(int, voidptr, usize) int
fn C.close(int) int

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

const max_read_buffer = 256
const max_write_buffer = 256

pub enum Baudrate {
	b0      = C.B0 // 0  [bps]
	b50     = C.B50 // 50 [bps]
	b75     = C.B75 // 75 [bps]
	b110    = C.B110 // 110 [bps]
	b134    = C.B134 // 134.5 [bps]
	b150    = C.B150 // 150 [bps]
	b200    = C.B200 // 200 [bps]
	b300    = C.B300 // 300 [bps]
	b600    = C.B600 // 600 [bps]
	b1200   = C.B1200 // 1200 [bps]
	b1800   = C.B1800 // 1800 [bps]
	b2400   = C.B2400 // 2400 [bps]
	b4800   = C.B4800 // 4800 [bps]
	b9600   = C.B9600 // 9600 [bps]
	b19200  = C.B19200 // 19200 [bps]
	b38400  = C.B38400 // 38400 [bps]
	b57600  = C.B57600 // 57,600 [bps]
	b115200 = C.B115200 // 115,200 [bps]
}

const c_baudrates = [0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 9600, 7200,
	14400, 19200, 28800, 38400]

pub enum Status {
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
	error_is_disconnected
}

pub fn (s Status)is_okay() bool {
	return bool(s == .okay)
}

pub enum Parity {
	@none
	odd
	even
}

pub enum FlowControl {
	hardware
	software
	@none
}

pub enum StopBit {
	one
	two
}

pub enum CharacterSize {
	char5b
	char6b
	char7b
	char8b
}

pub struct SerialPort {
mut:
	fd           int
	port_name    string
	baud_rate    Baudrate
	is_connected bool
	io_blocking  bool
	flow_control FlowControl
	parity       Parity
	stop_bit     StopBit
	char_size    CharacterSize

	lock_port bool = true

	flag_ignore_cr bool = true
	flag_map_cr_nl bool
	flag_map_nl_cr bool

	timeout u32 = 100 // [ms]
}

// new_default allocates a serial port with baudrate of 9600, no flow control,
// no parity, stop bit 1 and character size of 8 bits.
pub fn new_default(port_name string) SerialPort {
	sp := SerialPort{
		fd: -1
		baud_rate: .b9600
		flow_control: .@none
		parity: .@none
		stop_bit: .one
		char_size: .char8b
		port_name: port_name
	}

	return sp
}

// new requires port name, baudrate, flow control, parity, stop bits and the character size
// as inputs, in order to allocate the serial port.
pub fn new(port_name string, baud_rate Baudrate, flow_control FlowControl, parity Parity, stop_bit StopBit, char_size CharacterSize) SerialPort {
	sp := SerialPort{
		fd: -1
		baud_rate: baud_rate
		flow_control: flow_control
		parity: parity
		stop_bit: stop_bit
		char_size: char_size
		port_name: port_name
	}

	return sp
}

// open the serial port and configure it (if it is able to open the port).
// Returns enum of the type Status.
pub fn (mut sp SerialPort) open() Status {
	mut open_flag := u32(C.O_RDWR | C.O_NOCTTY)
	if sp.io_blocking == false {
		open_flag |= u32(C.O_NDELAY)
	} else {
		open_flag &= u32(~C.O_NDELAY)
	}

	sp.fd = int(C.open(sp.port_name.str, open_flag))
	if sp.fd == -1 {
		return sp.error(sp.fd)
	}
	sp.is_connected = true

	mut rc := C.isatty(sp.fd)
	if rc != 1 {
		sp.close()
		return Status.error_not_tty
	}

	if sp.lock_port == true {
		C.ioctl(sp.fd, C.TIOCEXCL, 0)
	} else {
		C.ioctl(sp.fd, C.TIOCGEXCL, 0)
	}

	rc = int(sp.flush())

	mut options := C.termios{}
	rc = int(C.tcgetattr(sp.fd, &options))
	if rc != 0 {
		sp.close()
		return Status.error_unknown
	}

	options.c_iflag |= u32((C.INPCK | C.ISTRIP))
	if sp.flag_ignore_cr == true {
		options.c_iflag &= u32(~C.IGNCR)
	} else {
		options.c_iflag |= u32(C.IGNCR)
	}

	if sp.flag_map_nl_cr == false {
		options.c_iflag &= u32(~C.INLCR)
	} else {
		options.c_iflag |= u32(C.INLCR)
	}

	if sp.flag_map_cr_nl == false {
		options.c_iflag &= u32(~C.ICRNL)
	} else {
		options.c_iflag |= u32(C.ICRNL)
	}

	options.c_oflag &= u32(~(C.ONLCR | C.OCRNL | C.OPOST))
	options.c_lflag &= u32(~(C.ECHO | C.ECHONL | C.ICANON | C.ISIG | C.IEXTEN))

	// Set up timeouts: Calls to read() will return as soon as there is
	// at least one byte available or when 100 ms has passed.
	options.c_cc[C.VTIME] = u8(((sp.timeout) / 100) & 0xFF)
	options.c_cc[C.VMIN] = 0

	// Set baudrate
	C.cfsetospeed(&options, int(sp.baud_rate))
	C.cfsetispeed(&options, int(sp.baud_rate))
	if int(C.cfgetospeed(&options)) != int(sp.baud_rate) {
		return Status.error_baudrate_mismatch
	}

	// set CFLAG
	options.c_cflag |= u32((C.CLOCAL | C.CREAD))

	// Configure flow control
	match sp.flow_control {
		.hardware {
			options.c_cflag &= u32(~C.CRTSCTS) // Clear CRTSCTS
			options.c_cflag |= u32(C.CRTSCTS)
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
		}
		.software {
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
			options.c_iflag |= u32((C.IXON | C.IXOFF | C.IXANY))
			options.c_cflag &= u32(~C.CRTSCTS)
		}
		else {
			options.c_iflag &= u32(~(C.IXON | C.IXOFF | C.IXANY))
			options.c_cflag &= u32(~C.CRTSCTS)
		}
	}

	// Configure parity
	options.c_iflag &= u32(~(C.INPCK | C.ISTRIP))
	options.c_cflag &= u32(~(C.PARENB | C.PARODD))
	match sp.parity {
		.@none {}
		.even {
			options.c_iflag |= u32((C.INPCK | C.ISTRIP))
			options.c_cflag |= u32(C.PARENB)
			options.c_cflag &= u32(~C.PARODD)
		}
		.odd {
			options.c_iflag |= u32((C.INPCK | C.ISTRIP))
			options.c_cflag |= u32(C.PARENB)
			options.c_cflag |= u32(C.PARODD)
		}
	}

	// Configure stop bit
	match sp.stop_bit {
		.one {
			options.c_cflag &= u32(~C.CSTOPB) // Clear stop bit (stop bit 1)
		}
		.two {
			options.c_cflag &= u32(~C.CSTOPB) // Clear stop bit (stop bit 1)
			options.c_cflag |= u32(C.CSTOPB) // Set stop bit (stop bit 2)
		}
	}

	// Configure character size
	options.c_cflag &= u32(~C.CSIZE) // Clear all the size bits
	options.c_cflag &= u32(~(C.CS5 | C.CS6 | C.CS7 | C.CS8))
	match sp.char_size {
		.char5b {
			options.c_cflag |= u32(C.CS5)
		}
		.char6b {
			options.c_cflag |= u32(C.CS6)
		}
		.char7b {
			options.c_cflag |= u32(C.CS7)
		}
		.char8b {
			options.c_cflag |= u32(C.CS8)
		}
	}

	// Set final configuration
	rc = int(C.tcsetattr(sp.fd, C.TCSANOW, &options))
	if rc != 0 {
		sp.close()
		return Status.error_configuration_failed
	}

	return Status.okay
}

// close the serial port.
pub fn (mut sp SerialPort) close() Status {
	if sp.is_connected == false {
		return Status.error_is_disconnected
	}
	return sp.error(C.close(sp.fd))
}

// connected returns true if the serial port is open (connected)
pub fn (mut sp SerialPort) is_connected() bool {
	return sp.is_connected
}

// available_bytes return number of availbale bytes in the input buffer.
pub fn (mut sp SerialPort) count() u32 {
	mut bytes_available := u32(0)

	rc := C.ioctl(sp.fd, C.FIONREAD, &bytes_available)
	if rc != 0 {
		return 0
	}
	return u32(bytes_available)
}

// empty returns false if there are byte(s) available in the input buffer
// otherwise, returns true
pub fn (mut sp SerialPort) is_empty() bool {
	return sp.count() == 0
}

// read received bytes and returns:
// number of received bytes (0 if no bytes avalable),
// read buffer - an empty buffer if no data, or an array of received bytes
// and the enum of type Status.
pub fn (mut sp SerialPort) read(maxbytes int) (int, []u8, Status) {
	C.fcntl(sp.fd, C.F_SETFL, C.FNDELAY)
	unsafe {
		mut buf := malloc_noscan(maxbytes + 1)
		nbytes := C.read(sp.fd, buf, maxbytes)
		if nbytes < 0 {
			free(buf)
			return 0, []u8{len: 0}, sp.error(nbytes)
		}
		buf[nbytes] = 0
		return nbytes, buf.vbytes(nbytes), Status.okay
	}
}

// read converts received bytes to a sting and returns:
// number of recieved bytes (0 if no data received),
// received string and the enum type of Status.
pub fn (mut sp SerialPort) read_string() (int, string, Status) {
	ncount, buffer, rc := sp.read(vserialx.max_read_buffer)

	mut rx_str := ''
	if ncount > 0 {
		rx_str = buffer.bytestr()
	}

	return ncount, rx_str, rc
}

// write_string sends a string and
// returns number of bytes that are sent and
// enum type from Status.
pub fn (mut sp SerialPort) write_string(package string) (u32, Status) {
	rc := int(C.write(sp.fd, package.str, usize(package.len)))

	if rc > 0 {
		return u32(rc), Status.okay
	}

	error := sp.error(rc)
	return 0, error
}

// write sends a byte array.
// Returns: number of written data and
// enum type from Status.
pub fn (mut sp SerialPort) write(package []u8) (u32, Status) {
	rc := int(C.write(sp.fd, voidptr(&package[0]), usize(package.len)))

	if rc > 0 {
		return u32(rc), Status.okay
	}

	error := sp.error(rc)
	return 0, error
}

// flush_in flushes received unread data
// Returns enum type from Status.
pub fn (mut sp SerialPort) flush_in() Status {
	rc := C.tcflush(sp.fd, C.TCIFLUSH)
	return sp.error(rc)
}

// flash_out removes written data that are not sent.
// Returns enum type from Status.
pub fn (mut sp SerialPort) flash_out() Status {
	rc := C.tcflush(sp.fd, C.TCOFLUSH)
	return sp.error(rc)
}

// flush removes both written data that are not sent and
// received unread data, and returns enum type from Status.
pub fn (mut sp SerialPort) flush() Status {
	rc := C.tcflush(sp.fd, C.TCIOFLUSH)
	return sp.error(rc)
}

pub fn (mut sp SerialPort) error(error_code int) Status {
	return match error_code {
		C.EBADF {
			Status.error_bad_file_descriptor
		}
		C.EDESTADDRREQ {
			Status.error_no_address
		}
		C.EFAULT {
			Status.error_buffer_fault
		}
		C.EFBIG {
			Status.error_buffer_overlimit
		}
		C.EINTR {
			Status.error_interrupted
		}
		C.EINVAL {
			Status.error_invalid_parameters
		}
		C.EIO {
			Status.error_io
		}
		C.ENOSPC {
			Status.error_no_space
		}
		C.EPERM {
			Status.error_no_permission
		}
		C.EPIPE {
			Status.error_closed_pipe
		}
		else {
			Status.error_unknown
		}
	}
}

// unlock_access acquires exclusive access of sp serial port.
pub fn (mut sp SerialPort) lock_access() {
	sp.lock_port = true
	C.ioctl(sp.fd, C.TIOCEXCL, 0)
}

// unlock_access removes exclusive access of sp serial port.
pub fn (mut sp SerialPort) unlock_access() {
	sp.lock_port = false
	C.ioctl(sp.fd, C.TIOCGEXCL, 0)
}

// is_locked checks if exclusive access is locked or not.
// Returns boolean: true if it is locked, otherwise false.
pub fn (mut sp SerialPort) is_locked() bool {
	return sp.lock_port
}

// baudrate reads actual (configured) baudrate.
// Return baudrate in bits-per-seconds, or it returns
// an error if it fails.
pub fn (mut sp SerialPort) baudrate() !int {
	mut tx_baudrate := int(0)
	mut rx_baudrate := int(0)
	mut options := C.termios{}

	rc := int(C.tcgetattr(sp.fd, &options))
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

	return vserialx.c_baudrates[tx_baudrate]
}
