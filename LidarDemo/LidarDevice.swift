//
//  LidarDevice.swift
//  LidarDemo
//

import Foundation
import os

enum LidarDeviceError: Error, CustomStringConvertible {
    case timeout(String)
    case unexpectedDescriptor(String)

    var description: String {
        switch self {
        case .timeout(let step): "No reply from lidar to \(step)"
        case .unexpectedDescriptor(let step): "Unexpected reply from lidar to \(step)"
        }
    }
}

/// Reads continuous scans from a Slamtec RPLIDAR C1 on a serial port.
final class LidarDevice: Sendable {
    static let baudRate = 460_800

    let path: String
    private static let logger = Logger(subsystem: "LidarDemo", category: "LidarDevice")

    init(path: String) {
        self.path = path
    }

    /// One `Lidar360Data` for each revolution, about 10 per second.
    /// When the caller stops the iteration, the lidar gets STOP and the port closes.
    func scans() -> AsyncThrowingStream<Lidar360Data, Error> {
        let path = self.path
        let isCancelled = OSAllocatedUnfairLock(initialState: false)

        return AsyncThrowingStream { continuation in
            continuation.onTermination = { _ in
                isCancelled.withLock { $0 = true }
            }

            // Blocking reads must not run on the Swift concurrency thread pool, so use a dedicated thread.
            let thread = Thread {
                do {
                    let port = try SerialPort(path: path, baudRate: Self.baudRate)
                    defer {
                        try? port.write(RPLidarCommand.stop.packet)
                        Thread.sleep(forTimeInterval: 0.01)
                        port.close()
                    }
                    try Self.startScan(on: port)

                    var assembler = ScanAssembler()
                    var buffer = [UInt8](repeating: 0, count: 4096)
                    while !isCancelled.withLock({ $0 }) {
                        let count = try port.read(into: &buffer)
                        for scan in assembler.append(buffer[0..<count]) {
                            continuation.yield(scan)
                        }
                    }
                    continuation.finish()
                } catch {
                    Self.logger.error("\(String(describing: error), privacy: .public)")
                    continuation.finish(throwing: error)
                }
            }
            thread.name = "LidarDevice \(path)"
            thread.qualityOfService = .userInitiated
            thread.start()
        }
    }

    /// Stops any scan that is in progress, logs the device info, then starts a standard SCAN.
    private static func startScan(on port: SerialPort) throws {
        try port.write(RPLidarCommand.stop.packet)
        // The lidar can send data for a short time after STOP. Wait, then discard it.
        Thread.sleep(forTimeInterval: 0.05)
        port.flush()

        try port.write(RPLidarCommand.getInfo.packet)
        let infoDescriptor = try readExactly(RPLidarResponseDescriptor.size, from: port, step: "GET_INFO")
        guard RPLidarResponseDescriptor(bytes: infoDescriptor) == .info else {
            throw LidarDeviceError.unexpectedDescriptor("GET_INFO")
        }
        let infoBytes = try readExactly(20, from: port, step: "GET_INFO")
        if let info = RPLidarDeviceInfo(bytes: infoBytes) {
            logger.info("Lidar on \(port.path, privacy: .public): \(info.description, privacy: .public)")
        }

        try port.write(RPLidarCommand.scan.packet)
        let scanDescriptor = try readExactly(RPLidarResponseDescriptor.size, from: port, step: "SCAN")
        guard RPLidarResponseDescriptor(bytes: scanDescriptor) == .scan else {
            throw LidarDeviceError.unexpectedDescriptor("SCAN")
        }
    }

    private static func readExactly(_ count: Int, from port: SerialPort, step: String,
                                    timeout: TimeInterval = 1.0) throws -> [UInt8] {
        var result: [UInt8] = []
        var buffer = [UInt8](repeating: 0, count: count)
        let deadline = Date.now.addingTimeInterval(timeout)
        while result.count < count {
            guard Date.now < deadline else { throw LidarDeviceError.timeout(step) }
            buffer = [UInt8](repeating: 0, count: count - result.count)
            let read = try port.read(into: &buffer)
            result.append(contentsOf: buffer[0..<read])
        }
        return result
    }
}
