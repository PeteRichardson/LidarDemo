//
//  LidarDemoTests.swift
//  LidarDemoTests
//
//  Created by Peter Richardson on 11/8/26.
//

import SwiftUI
import Testing
@testable import LidarDemo

/// Encodes one standard SCAN node, the opposite of `RPLidarScanNode.decode`.
private func nodeBytes(start: Bool, quality: UInt8 = 47, angleQ6: UInt16, distanceQ2: UInt16) -> [UInt8] {
    [
        quality << 2 | (start ? 0b01 : 0b10),
        UInt8(angleQ6 & 0x7F) << 1 | 1,
        UInt8(angleQ6 >> 7),
        UInt8(distanceQ2 & 0xFF),
        UInt8(distanceQ2 >> 8),
    ]
}

/// Encodes a node from degrees and millimetres.
private func node(start: Bool = false, degrees: Double, mm: Double) -> [UInt8] {
    nodeBytes(start: start, angleQ6: UInt16(degrees * 64), distanceQ2: UInt16(mm * 4))
}

struct RPLidarProtocolTests {

    @Test func commandPackets() {
        #expect(RPLidarCommand.stop.packet == [0xA5, 0x25])
        #expect(RPLidarCommand.scan.packet == [0xA5, 0x20])
        #expect(RPLidarCommand.getInfo.packet == [0xA5, 0x50])
    }

    @Test func scanDescriptor() {
        let bytes: [UInt8] = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81]
        #expect(RPLidarResponseDescriptor(bytes: bytes) == .scan)
    }

    @Test func infoDescriptor() {
        let bytes: [UInt8] = [0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04]
        #expect(RPLidarResponseDescriptor(bytes: bytes) == .info)
    }

    @Test func descriptorRejectsBadSyncOrLength() {
        #expect(RPLidarResponseDescriptor(bytes: [0xA5, 0x00, 0x05, 0x00, 0x00, 0x40, 0x81]) == nil)
        #expect(RPLidarResponseDescriptor(bytes: [0xA5, 0x5A, 0x05]) == nil)
    }

    @Test func decodeNode() throws {
        let decoded = try #require(RPLidarScanNode.decode(node(start: true, degrees: 90, mm: 1000)))
        #expect(decoded.isStartOfScan)
        #expect(decoded.quality == 47)
        #expect(decoded.angleDegrees == 90)
        #expect(decoded.distanceQ2 == 4000)
    }

    @Test func decodeRejectsBadStartBits() {
        var bytes = node(degrees: 10, mm: 500)
        bytes[0] |= 0b11  // S and !S both 1
        #expect(RPLidarScanNode.decode(bytes) == nil)
    }

    @Test func decodeRejectsMissingCheckBit() {
        var bytes = node(degrees: 10, mm: 500)
        bytes[1] &= 0xFE
        #expect(RPLidarScanNode.decode(bytes) == nil)
    }
}

struct ScanAssemblerTests {

    @Test func outputsRevolutionWhenNextOneStarts() {
        var assembler = ScanAssembler()
        let first = node(start: true, degrees: 0, mm: 100) + node(degrees: 120, mm: 200) + node(degrees: 240, mm: 300)
        #expect(assembler.append(first).isEmpty)

        let scans = assembler.append(node(start: true, degrees: 1, mm: 400))
        #expect(scans.count == 1)
        #expect(scans.first?.map(\.dist) == [100, 200, 300])
    }

    @Test func discardsNodesBeforeFirstStart() {
        var assembler = ScanAssembler()
        let bytes = node(degrees: 300, mm: 999) + node(start: true, degrees: 0, mm: 100)
            + node(degrees: 180, mm: 200) + node(start: true, degrees: 0, mm: 100)
        let scans = assembler.append(bytes)
        #expect(scans.map { $0.map(\.dist) } == [[100, 200]])
    }

    @Test func dropsZeroDistance() {
        var assembler = ScanAssembler()
        let bytes = node(start: true, degrees: 0, mm: 100) + node(degrees: 90, mm: 0)
            + node(degrees: 180, mm: 200) + node(start: true, degrees: 0, mm: 100)
        #expect(assembler.append(bytes).first?.count == 2)
    }

    @Test func joinsNodesSplitAcrossReads() {
        var assembler = ScanAssembler()
        let bytes = node(start: true, degrees: 0, mm: 100) + node(degrees: 180, mm: 200)
            + node(start: true, degrees: 0, mm: 100)
        var scans: [Lidar360Data] = []
        for byte in bytes {
            scans += assembler.append([byte])
        }
        #expect(scans.map { $0.map(\.dist) } == [[100, 200]])
    }

    @Test func syncsAgainAfterBadBytes() {
        var assembler = ScanAssembler()
        let bytes = [0x00, 0x00] + node(start: true, degrees: 0, mm: 100) + node(degrees: 180, mm: 200)
            + node(start: true, degrees: 0, mm: 100)
        let scans = assembler.append(bytes)
        #expect(scans.map { $0.map(\.dist) } == [[100, 200]])
        #expect(assembler.discardedBytes == 2)
    }

    @Test func keepsAngle() throws {
        var assembler = ScanAssembler()
        let bytes = node(start: true, degrees: 45, mm: 100) + node(start: true, degrees: 0, mm: 100)
        let angle = try #require(assembler.append(bytes).first?.first?.angle)
        #expect(abs(angle - 45) < 0.01)
    }
}

struct Lidar360PlotTests {
    let rect = CGRect(x: 0, y: 0, width: 200, height: 200)

    @Test func emptyScanGivesEmptyPath() {
        #expect(Lidar360Plot(nodes: [], rangeMM: 2000).path(in: rect).isEmpty)
    }

    @Test(arguments: [
        (degrees: 0.0, x: 100.0, y: 0.0),     // arrow direction at the top
        (degrees: 90.0, x: 200.0, y: 100.0),  // clockwise, so 90° at the right
        (degrees: 180.0, x: 100.0, y: 200.0),
        (degrees: 270.0, x: 0.0, y: 100.0),
    ])
    func rangeMapsToEdgeOfPlot(degrees: Double, x: Double, y: Double) {
        let plot = Lidar360Plot(nodes: [], rangeMM: 2000)
        let center = CGPoint(x: rect.midX, y: rect.midY)
        let point = plot.toPoint(LidarNode(degrees, 2000 * 4, 0, 0), center: center, scale: 100 / 2000)
        #expect(abs(point.x - x) < 0.01)
        #expect(abs(point.y - y) < 0.01)
    }

    @Test func nodesOutOfRangeStayInsidePlot() {
        let nodes = [LidarNode(0, 10_000 * 4, 0, 0), LidarNode(90, 10_000 * 4, 0, 0), LidarNode(180, 500 * 4, 0, 0)]
        let bounds = Lidar360Plot(nodes: nodes, rangeMM: 2000).path(in: rect).boundingRect
        // These nodes are far apart, so each one is a dot that can extend past the edge by its radius.
        let margin = Lidar360Plot.dotRadius + 0.001
        #expect(rect.insetBy(dx: -margin, dy: -margin).contains(bounds))
    }

    /// Nodes every 1° at the given ranges in millimetres, starting at 0°.
    private func scan(_ ranges: [Double]) -> Lidar360Data {
        ranges.enumerated().map { LidarNode(Double($0.offset), UInt32($0.element * 4), 0, 0) }
    }

    @Test func nearObjectIsSeparateFromWall() {
        let nodes = scan([1000, 1000, 1000, 500, 500, 500, 1000, 1000, 1000])
        let (segments, closed) = Lidar360Plot.segments(nodes, toleranceMM: 30)
        #expect(segments.map(\.count) == [3, 3, 3])
        #expect(!closed)
    }

    @Test func smallNoiseStaysOneSurface() {
        let nodes = scan([1000, 1010, 995, 1005, 1000])
        #expect(Lidar360Plot.segments(nodes, toleranceMM: 30).segments.count == 1)
    }

    @Test func toleranceIncreasesWithRange() {
        // Beams 1° apart are 87 mm apart at 5 m. The same range step at 5 m is one surface, but not at 0.5 m.
        let far = (LidarNode(0, 5000 * 4, 0, 0), LidarNode(1, 5100 * 4, 0, 0))
        let near = (LidarNode(0, 500 * 4, 0, 0), LidarNode(1, 600 * 4, 0, 0))
        let step = Double.pi / 180
        #expect(Lidar360Plot.isSameSurface(far.0, far.1, step: step, toleranceMM: 30))
        #expect(!Lidar360Plot.isSameSurface(near.0, near.1, step: step, toleranceMM: 30))
    }

    @Test func missingReadingsBreakSurface() {
        let nodes = [LidarNode(0, 1000 * 4, 0, 0), LidarNode(20, 1000 * 4, 0, 0)]
        #expect(Lidar360Plot.segments(nodes, toleranceMM: 30).segments.count == 2)
    }

    @Test func fullRingIsClosed() {
        let nodes = (0..<360).map { LidarNode(Double($0), 1000 * 4, 0, 0) }
        let (segments, closed) = Lidar360Plot.segments(nodes, toleranceMM: 30)
        #expect(segments.count == 1)
        #expect(closed)
    }

    @Test func surfaceAcross360IsJoined() {
        // A near object from 355° to 4°, on a wall at 1 m.
        let nodes = (0..<360).map { LidarNode(Double($0), ($0 < 5 || $0 >= 355 ? 500 : 1000) * 4, 0, 0) }
        let (segments, closed) = Lidar360Plot.segments(nodes, toleranceMM: 30)
        #expect(segments.map(\.count) == [10, 350])
        #expect(segments.first?.first?.angle ?? 0 > 354)
        #expect(!closed)
    }

    @Test func sampleScanFitsInPlot() {
        let bounds = Lidar360Plot(nodes: sample360Data, rangeMM: 2000).path(in: rect).boundingRect
        #expect(!bounds.isEmpty)
        #expect(rect.contains(bounds))
    }
}
