//
//  SerialPort.swift
//  LidarDemo
//

import Foundation

struct SerialPortError: Error, CustomStringConvertible {
    let operation: String
    let code: Int32

    var description: String { "\(operation) failed: \(String(cString: strerror(code))) (errno \(code))" }
}

/// A raw 8N1 serial port with no flow control. Reads block for at most 100 ms.
final class SerialPort {
    /// IOSSIOSPEED from <IOKit/serial/ioss.h> is `_IOW('T', 2, speed_t)`. Swift cannot import that macro.
    /// speed_t is 8 bytes on 64-bit macOS, so the value is 0x80085402.
    private static let iossioSpeed: UInt = 0x8008_5402

    let path: String
    private var fd: Int32 = -1

    init(path: String, baudRate: Int) throws {
        self.path = path

        // O_NONBLOCK so that open() does not wait for carrier detect. Clear it after open.
        fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK)
        guard fd >= 0 else { throw SerialPortError(operation: "open \(path)", code: errno) }

        do {
            // Exclusive access, so a second app cannot read the same bytes.
            try check(ioctl(fd, TIOCEXCL), "TIOCEXCL")
            try check(fcntl(fd, F_SETFL, 0), "fcntl")

            var options = termios()
            try check(tcgetattr(fd, &options), "tcgetattr")
            cfmakeraw(&options)
            options.c_cflag |= tcflag_t(CLOCAL | CREAD)
            options.c_cflag &= ~tcflag_t(CRTSCTS | PARENB | CSTOPB)
            // VMIN = 0, VTIME = 1: read() returns when data arrives, or after 100 ms with 0 bytes.
            withUnsafeMutableBytes(of: &options.c_cc) { cc in
                cc[Int(VMIN)] = 0
                cc[Int(VTIME)] = 1
            }
            try check(tcsetattr(fd, TCSANOW, &options), "tcsetattr")

            // 460800 is not a standard termios speed on macOS. Set it after tcsetattr.
            var speed = speed_t(baudRate)
            try check(ioctl(fd, Self.iossioSpeed, &speed), "IOSSIOSPEED")
        } catch {
            Darwin.close(fd)
            fd = -1
            throw error
        }
    }

    deinit { close() }

    func write(_ bytes: [UInt8]) throws {
        var offset = 0
        while offset < bytes.count {
            let written = bytes[offset...].withUnsafeBytes { Darwin.write(fd, $0.baseAddress, $0.count) }
            guard written >= 0 else { throw SerialPortError(operation: "write", code: errno) }
            offset += written
        }
    }

    /// Reads up to `buffer.count` bytes. Returns 0 if no data came in 100 ms.
    func read(into buffer: inout [UInt8]) throws -> Int {
        let count = buffer.withUnsafeMutableBytes { Darwin.read(fd, $0.baseAddress, $0.count) }
        guard count >= 0 else { throw SerialPortError(operation: "read", code: errno) }
        return count
    }

    /// Discards bytes in the input and output buffers.
    func flush() {
        tcflush(fd, TCIOFLUSH)
    }

    func close() {
        guard fd >= 0 else { return }
        Darwin.close(fd)
        fd = -1
    }

    private func check(_ result: Int32, _ operation: String) throws {
        guard result != -1 else { throw SerialPortError(operation: operation, code: errno) }
    }
}
