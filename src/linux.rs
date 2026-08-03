#![allow(non_camel_case_types)]

use std::ffi::c_void;
use std::os::raw::{c_int, c_ulong};

pub type tcflag_t = u32;
pub type cc_t = u8;
pub type speed_t = u32;

pub const O_NOCTTY: i32 = 0o400;
pub const O_NONBLOCK: i32 = 0o4000;

pub const NCCS: usize = 32;
pub const B9600: speed_t = 0o0000015;
pub const B57600: speed_t = 0o0010001;
pub const B115200: speed_t = 0o0010002;
pub const B230400: speed_t = 0o0010003;

pub const IGNBRK: tcflag_t = 0o0000001;
pub const BRKINT: tcflag_t = 0o0000002;
pub const PARMRK: tcflag_t = 0o0000010;
pub const ISTRIP: tcflag_t = 0o0000040;
pub const INLCR: tcflag_t = 0o0000100;
pub const IGNCR: tcflag_t = 0o0000200;
pub const ICRNL: tcflag_t = 0o0000400;
pub const IXON: tcflag_t = 0o0002000;
pub const IXOFF: tcflag_t = 0o0010000;
pub const IXANY: tcflag_t = 0o0004000;

pub const OPOST: tcflag_t = 0o0000001;

pub const CSIZE: tcflag_t = 0o0000060;
pub const CS8: tcflag_t = 0o0000060;
pub const CSTOPB: tcflag_t = 0o0000100;
pub const CREAD: tcflag_t = 0o0000200;
pub const PARENB: tcflag_t = 0o0000400;
pub const PARODD: tcflag_t = 0o0001000;
pub const CLOCAL: tcflag_t = 0o0004000;
pub const CRTSCTS: tcflag_t = 0o20000000000;

pub const TCSANOW: c_int = 0;
pub const TCIFLUSH: c_int = 0;
pub const TCIOFLUSH: c_int = 2;

pub const SIGINT: c_int = 2;
pub const SIGTERM: c_int = 15;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Termios {
    pub c_iflag: tcflag_t,
    pub c_oflag: tcflag_t,
    pub c_cflag: tcflag_t,
    pub c_lflag: tcflag_t,
    pub c_line: cc_t,
    pub c_cc: [cc_t; NCCS],
    pub c_ispeed: speed_t,
    pub c_ospeed: speed_t,
}

pub type SignalHandler = extern "C" fn(c_int);

unsafe extern "C" {
    pub fn ioctl(fd: c_int, request: c_ulong, ...) -> c_int;
    pub fn tcgetattr(fd: c_int, termios_p: *mut Termios) -> c_int;
    pub fn tcsetattr(fd: c_int, optional_actions: c_int, termios_p: *const Termios) -> c_int;
    pub fn cfsetispeed(termios_p: *mut Termios, speed: speed_t) -> c_int;
    pub fn cfsetospeed(termios_p: *mut Termios, speed: speed_t) -> c_int;
    pub fn tcflush(fd: c_int, queue_selector: c_int) -> c_int;
    pub fn tcdrain(fd: c_int) -> c_int;
    pub fn signal(signum: c_int, handler: SignalHandler) -> SignalHandler;
    pub fn memset(s: *mut c_void, c: c_int, n: usize) -> *mut c_void;
}
