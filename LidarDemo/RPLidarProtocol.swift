//
//  RPLidarProtocol.swift
//  LidarDemo
//
//  The RPLIDAR serial protocol: request packets, response descriptors and
//  standard SCAN measurement nodes. No I/O here, so the code is easy to test.
//

import Foundation

enum RPLidarCommand: UInt8 {
    case stop = 0x25
    case reset = 0x40
    case scan = 0x20
    case getInfo = 0x50
    case getHealth = 0x52

    /// Request packet without payload: sync byte 0xA5, then the command.
    var packet: [UInt8] { [0xA5, rawValue] }
}

/// The 7-byte header that comes before every response: A5 5A, then 30-bit length + 2-bit mode, then data type.
struct RPLidarResponseDescriptor: Equatable {
    static let size = 7

    let length: UInt32
    let mode: UInt8
    let dataType: UInt8

    /// SCAN replies with 5-byte nodes, mode 1 (multiple responses), type 0x81.
    static let scan = RPLidarResponseDescriptor(length: 5, mode: 1, dataType: 0x81)
    /// GET_INFO replies with one 20-byte response, type 0x04.
    static let info = RPLidarResponseDescriptor(length: 20, mode: 0, dataType: 0x04)

    init(length: UInt32, mode: UInt8, dataType: UInt8) {
        self.length = length
        self.mode = mode
        self.dataType = dataType
    }

    init?(bytes: some Collection<UInt8>) {
        let b = Array(bytes)
        guard b.count == Self.size, b[0] == 0xA5, b[1] == 0x5A else { return nil }
        let word = UInt32(b[2]) | UInt32(b[3]) << 8 | UInt32(b[4]) << 16 | UInt32(b[5]) << 24
        length = word & 0x3FFF_FFFF
        mode = UInt8(word >> 30)
        dataType = b[6]
    }
}

/// The payload of a GET_INFO response.
struct RPLidarDeviceInfo: CustomStringConvertible {
    let model: UInt8
    let firmwareMinor: UInt8
    let firmwareMajor: UInt8
    let hardware: UInt8
    let serialNumber: [UInt8]

    init?(bytes: some Collection<UInt8>) {
        let b = Array(bytes)
        guard b.count == 20 else { return nil }
        model = b[0]
        firmwareMinor = b[1]
        firmwareMajor = b[2]
        hardware = b[3]
        serialNumber = Array(b[4..<20])
    }

    var description: String {
        let serial = serialNumber.map { String(format: "%02X", $0) }.joined()
        return String(format: "model 0x%02X, firmware %d.%02d, hardware %d, serial %@",
                      model, firmwareMajor, firmwareMinor, hardware, serial)
    }
}

/// Decodes the 5-byte nodes of a standard SCAN.
///
/// Byte 0: bit 0 = S (start of new revolution), bit 1 = !S, bits 2-7 = quality.
/// Byte 1: bit 0 = check bit (always 1), bits 1-7 = angle_q6 bits 0-6.
/// Byte 2: angle_q6 bits 7-14. Angle in degrees = angle_q6 / 64.
/// Bytes 3-4: distance_q2, little-endian. Distance in mm = distance_q2 / 4.
enum RPLidarScanNode {
    static let size = 5

    struct Decoded: Equatable {
        let isStartOfScan: Bool
        let quality: UInt8
        let angleDegrees: Double
        let distanceQ2: UInt16
    }

    /// Returns nil if the bytes fail the sync and check bit tests.
    static func decode(_ bytes: some Collection<UInt8>) -> Decoded? {
        let b = Array(bytes)
        guard b.count == size else { return nil }
        let start = b[0] & 0x01
        let inverseStart = (b[0] >> 1) & 0x01
        guard start != inverseStart, b[1] & 0x01 == 1 else { return nil }

        let angleQ6 = UInt16(b[1] >> 1) | UInt16(b[2]) << 7
        let distanceQ2 = UInt16(b[3]) | UInt16(b[4]) << 8
        return Decoded(isStartOfScan: start == 1,
                       quality: b[0] >> 2,
                       angleDegrees: Double(angleQ6) / 64.0,
                       distanceQ2: distanceQ2)
    }
}

/// Collects bytes of a SCAN stream and outputs one `Lidar360Data` for each full revolution.
struct ScanAssembler {
    private var pending: [UInt8] = []
    private var current: Lidar360Data = []
    /// False until the first start-of-scan node. Nodes before it are a part revolution.
    private var hasStart = false

    /// Count of bytes discarded to get back in sync. Useful to find problems with the link.
    private(set) var discardedBytes = 0

    /// Adds bytes and returns all revolutions that they complete.
    mutating func append(_ bytes: some Collection<UInt8>) -> [Lidar360Data] {
        pending.append(contentsOf: bytes)
        var scans: [Lidar360Data] = []
        var index = 0

        while pending.count - index >= RPLidarScanNode.size {
            guard let node = RPLidarScanNode.decode(pending[index..<index + RPLidarScanNode.size]) else {
                // Not a valid node at this position. Move one byte and try again.
                index += 1
                discardedBytes += 1
                continue
            }
            index += RPLidarScanNode.size

            if node.isStartOfScan {
                if hasStart, !current.isEmpty {
                    scans.append(current)
                }
                current = []
                hasStart = true
            }
            // Distance 0 means the lidar got no return at this angle.
            guard hasStart, node.distanceQ2 > 0 else { continue }
            current.append(LidarNode(node.angleDegrees, UInt32(node.distanceQ2), node.quality,
                                     node.isStartOfScan ? 1 : 0))
        }

        pending.removeFirst(index)
        return scans
    }
}
