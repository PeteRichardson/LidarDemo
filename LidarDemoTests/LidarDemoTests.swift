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

struct LidarItemTests {
    /// Nodes every 1° from `start`, all at one range in millimetres.
    private func arc(from start: Int, count: Int, mm: Double) -> Lidar360Data {
        (start..<start + count).map { LidarNode(Double($0), UInt32(mm * 4), 0, 0) }
    }

    @Test func rangeRunsShareBoundaryNode() {
        let nodes = [LidarNode(0, 400 * 4, 0, 0), LidarNode(1, 600 * 4, 0, 0), LidarNode(2, 600 * 4, 0, 0), LidarNode(3, 400 * 4, 0, 0)]
        let runs = Lidar360Plot.rangeRuns(nodes, rangeMM: 500)
        #expect(runs.map(\.beyondRange) == [false, true, false])
        #expect(runs.map { $0.nodes.map(\.dist) } == [[400], [400, 600, 600, 400], [400]])
    }

    @Test func itemKeepsColorWhenItMovesALittle() {
        var tracker = LidarItemTracker()
        let first = tracker.items(in: arc(from: 10, count: 5, mm: 500) + arc(from: 90, count: 5, mm: 500), toleranceMM: 30)
        let second = tracker.items(in: arc(from: 11, count: 5, mm: 490) + arc(from: 91, count: 5, mm: 510), toleranceMM: 30)
        #expect(first.map(\.colorIndex) == [0, 1])
        #expect(second.map(\.colorIndex) == [0, 1])
    }

    @Test func newItemGetsNewColor() {
        var tracker = LidarItemTracker()
        _ = tracker.items(in: arc(from: 10, count: 5, mm: 500), toleranceMM: 30)
        let items = tracker.items(in: arc(from: 10, count: 5, mm: 500) + arc(from: 180, count: 5, mm: 500), toleranceMM: 30)
        #expect(items.map(\.colorIndex) == [0, 1])
    }

    @Test func itemMovingAwayHasOutwardVelocity() throws {
        var tracker = LidarItemTracker()
        _ = tracker.items(in: arc(from: 80, count: 21, mm: 500), toleranceMM: 30, time: 0)
        let items = tracker.items(in: arc(from: 80, count: 21, mm: 550), toleranceMM: 30, time: 0.1)
        // The item is too large for a rectangle fit. It is at 90°, to the right of the lidar, and moved 50 mm in 0.1 s: 500 mm/s before smoothing.
        let motion = try #require(items.first?.motion)
        #expect(abs(motion.velocity.dx - 500 * LidarItemTracker.velocitySmoothing) < 1)
        #expect(abs(motion.velocity.dy) < 1)
    }

    @Test func sameScanAgainKeepsVelocity() throws {
        var tracker = LidarItemTracker()
        _ = tracker.items(in: arc(from: 80, count: 21, mm: 500), toleranceMM: 30, time: 0)
        let first = try #require(tracker.items(in: arc(from: 80, count: 21, mm: 550), toleranceMM: 30, time: 0.1).first?.motion)
        let again = try #require(tracker.items(in: arc(from: 80, count: 21, mm: 550), toleranceMM: 30, time: 0.1).first?.motion)
        #expect(again.velocity == first.velocity)
    }

    @Test func fastItemKeepsColorFromPrediction() {
        // The second step is 230 mm, more than maxMoveMM. It matches only because the tracker predicts the position from the speed.
        var tracker = LidarItemTracker()
        _ = tracker.items(in: arc(from: 80, count: 21, mm: 500), toleranceMM: 30, time: 0)
        _ = tracker.items(in: arc(from: 80, count: 21, mm: 680), toleranceMM: 30, time: 0.1)
        let items = tracker.items(in: arc(from: 80, count: 21, mm: 910), toleranceMM: 30, time: 0.2)
        #expect(items.map(\.colorIndex) == [0])
    }

    @Test func singleNodeHasNoColor() {
        var tracker = LidarItemTracker()
        let items = tracker.items(in: [LidarNode(10, 500 * 4, 0, 0)], toleranceMM: 30)
        #expect(items.map(\.colorIndex) == [nil])
    }
}

struct LidarBackgroundTests {
    /// One node at the centre of each 1° bin, at the given range in millimetres.
    private func ring(mm: Double, except object: (bins: Range<Int>, mm: Double)? = nil) -> Lidar360Data {
        (0..<360).map { bin in
            let range = object.map { $0.bins.contains(bin) ? $0.mm : mm } ?? mm
            return LidarNode(Double(bin) + 0.5, UInt32(range * 4), 0, 0)
        }
    }

    /// Learns the scan as one sample. A sample is taken only every `scansPerSample` scans.
    private func learnSample(_ background: inout LidarBackground, _ scan: Lidar360Data) {
        for _ in 0..<LidarBackground.scansPerSample {
            background.learn(scan)
        }
    }

    @Test func emptyBackgroundMakesAllForeground() {
        #expect(LidarBackground().isForeground(LidarNode(10.5, 1000 * 4, 0, 0)))
    }

    @Test func firstScanIsBackground() {
        var background = LidarBackground()
        background.learn(ring(mm: 1000))
        #expect(!background.isForeground(LidarNode(10.5, 1000 * 4, 0, 0)))
        #expect(!background.isForeground(LidarNode(10.5, 970 * 4, 0, 0)))  // inside the margin
    }

    @Test func nearerObjectIsForeground() {
        var background = LidarBackground()
        background.learn(ring(mm: 1000))
        #expect(background.isForeground(LidarNode(10.5, 500 * 4, 0, 0)))
    }

    @Test func adjacentBinsPreventEdgeFlicker() {
        // An object at 500 mm in bins 10..<20. Bin 20 is next to the object, so a 500 mm reading there is not foreground.
        var background = LidarBackground()
        background.learn(ring(mm: 1000, except: (10..<20, 500)))
        #expect(!background.isForeground(LidarNode(20.5, 500 * 4, 0, 0)))
        #expect(background.isForeground(LidarNode(22.5, 500 * 4, 0, 0)))
    }

    @Test func objectThatStaysBecomesBackground() {
        var background = LidarBackground()
        learnSample(&background, ring(mm: 1000))
        let withObject = ring(mm: 1000, except: (10..<20, 500))
        for _ in 0..<LidarBackground.sampleCount / 2 {
            learnSample(&background, withObject)
        }
        #expect(!background.isForeground(LidarNode(15.5, 500 * 4, 0, 0)))
    }

    @Test func backgroundItemsHaveNoColor() {
        var background = LidarBackground()
        background.learn(ring(mm: 1000))
        var tracker = LidarItemTracker()
        let items = tracker.items(in: ring(mm: 1000, except: (10..<20, 500)), toleranceMM: 30, background: background)
        #expect(items.compactMap(\.colorIndex).count == 1)
        #expect(items.first { $0.colorIndex != nil }?.nodes.allSatisfy { $0.dist == 500 } == true)
    }
}

struct RectangleFitTests {
    /// What the lidar sees of a rectangle with this pose: the nearest hit of each beam, with beams `stepDegrees` apart.
    private func scan(_ pose: RectangleFit.Pose, stepDegrees: Double = 1) -> [CGPoint] {
        let (u, v) = pose.axes
        let half = (u: RectangleFit.lengthMM / 2, v: RectangleFit.widthMM / 2)
        // The lidar's position in the rectangle's frame.
        let origin = (u: -(pose.center.x * u.dx + pose.center.y * u.dy), v: -(pose.center.x * v.dx + pose.center.y * v.dy))
        return stride(from: 0.0, to: 360, by: stepDegrees).compactMap { degrees in
            let radians = degrees * .pi / 180
            let direction = CGVector(dx: sin(radians), dy: cos(radians))
            let d = (u: direction.dx * u.dx + direction.dy * u.dy, v: direction.dx * v.dx + direction.dy * v.dy)
            // Slab method: the beam is inside the rectangle between tNear and tFar.
            var tNear = -CGFloat.infinity
            var tFar = CGFloat.infinity
            for (o, dd, h) in [(origin.u, d.u, half.u), (origin.v, d.v, half.v)] {
                if abs(dd) < 1e-12 {
                    if abs(o) > h { return nil }
                    continue
                }
                let t1 = (-h - o) / dd
                let t2 = (h - o) / dd
                tNear = max(tNear, min(t1, t2))
                tFar = min(tFar, max(t1, t2))
            }
            guard tNear <= tFar, tNear > 0 else { return nil }
            return CGPoint(x: direction.dx * tNear, y: direction.dy * tNear)
        }
    }

    private func expectClose(_ fitted: RectangleFit.Pose?, _ truth: RectangleFit.Pose, sourceLocation: SourceLocation = #_sourceLocation) {
        guard let fitted else {
            Issue.record("No fit", sourceLocation: sourceLocation)
            return
        }
        #expect(hypot(fitted.center.x - truth.center.x, fitted.center.y - truth.center.y) < 3, sourceLocation: sourceLocation)
        #expect(abs(remainder(fitted.angle - truth.angle, .pi)) < 2 * .pi / 180, sourceLocation: sourceLocation)
    }

    @Test(arguments: [0.0, 20, 45, 70, 90, 130, 160])
    func findsPoseWhenTwoFacesAreVisible(degrees: Double) {
        let truth = RectangleFit.Pose(center: CGPoint(x: 120, y: 300), angle: degrees * .pi / 180)
        expectClose(RectangleFit.fit(scan(truth, stepDegrees: 0.5)), truth)
    }

    @Test func findsPoseWhenOnlyLongFaceIsVisible() {
        // The long side is across the beam, straight in front of the lidar.
        let truth = RectangleFit.Pose(center: CGPoint(x: 0, y: 300), angle: 0)
        expectClose(RectangleFit.fit(scan(truth, stepDegrees: 0.5)), truth)
    }

    @Test func rejectsLargeItem() {
        let wall = (0..<20).map { CGPoint(x: CGFloat($0) * 10, y: 500) }
        #expect(RectangleFit.fit(wall) == nil)
    }

    @Test func smoothingTurnsTheShortWay() {
        // From 175° to 5° is a 10° turn through 180°, not a 170° turn back.
        let old = RectangleFit.Pose(center: .zero, angle: 175 * .pi / 180)
        let measured = RectangleFit.Pose(center: .zero, angle: 5 * .pi / 180)
        let smoothed = old.smoothed(toward: measured, shift: .zero, weight: 0.5)
        #expect(abs(smoothed.angle) < 0.001 || abs(smoothed.angle - .pi) < 0.001)
    }
}
