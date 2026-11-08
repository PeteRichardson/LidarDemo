//
//  LidarBackground.swift
//  LidarDemo
//

import Foundation

/// Learns the usual range at each angle, so that the model can find readings that are nearer than usual.
/// The lidar must not move. If it moves, make a new background.
struct LidarBackground {
    /// Number of angle bins, 1° each.
    static let binCount = 360
    /// Scans between two samples. The C1 makes approximately 10 scans each second, so this is one sample each second.
    static let scansPerSample = 10
    /// Samples kept for each bin. An object that stays for more than half of these samples becomes background.
    static let sampleCount = 30
    /// A reading must be this much nearer than the background to be foreground.
    static let marginMM: CGFloat = 50

    /// Recent samples for each bin, oldest first. Infinity means that there was no return.
    private var history: [[CGFloat]] = Array(repeating: [], count: binCount)
    /// Median of the history for each bin. Infinity until the bin has a sample.
    private(set) var ranges: [CGFloat] = Array(repeating: .infinity, count: binCount)
    private var scansSinceSample = 0

    static func bin(_ node: LidarNode) -> Int {
        Int(node.angle) % binCount
    }

    /// Adds a sample from every `scansPerSample` scans, starting with the first scan.
    mutating func learn(_ scan: Lidar360Data) {
        defer { scansSinceSample = (scansSinceSample + 1) % Self.scansPerSample }
        guard scansSinceSample == 0, !scan.isEmpty else { return }

        // Use the farthest reading in each bin. Nearer readings are possibly an object in front of the background.
        var sample = [CGFloat](repeating: -.infinity, count: Self.binCount)
        for node in scan {
            sample[Self.bin(node)] = max(sample[Self.bin(node)], node.dist)
        }
        for bin in 0..<Self.binCount {
            history[bin].append(sample[bin] == -.infinity ? .infinity : sample[bin])
            if history[bin].count > Self.sampleCount {
                history[bin].removeFirst()
            }
            let sorted = history[bin].sorted()
            ranges[bin] = sorted[sorted.count / 2]
        }
    }

    /// True if the node is nearer than the background in its bin and in the two adjacent bins.
    /// The adjacent bins prevent flicker at the edge of an object, where a bin contains both near and far readings.
    func isForeground(_ node: LidarNode) -> Bool {
        let bin = Self.bin(node)
        let neighbours = [(bin + Self.binCount - 1) % Self.binCount, bin, (bin + 1) % Self.binCount]
        let background = neighbours.map { ranges[$0] }.min() ?? .infinity
        return node.dist < background - Self.marginMM
    }
}
