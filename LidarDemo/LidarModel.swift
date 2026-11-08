//
//  LidarModel.swift
//  LidarDemo
//

import Foundation
import Observation

/// Holds the latest lidar scan for the UI. Finds the lidar, reads scans, and connects again after an error.
@Observable
@MainActor
final class LidarModel {
    enum Status: Equatable {
        case sample
        case searching
        case connected(String)
        case error(String)
    }

    private(set) var scan: Lidar360Data {
        didSet {
            scanTime = ProcessInfo.processInfo.systemUptime
            background.learn(scan)
            updateItems()
        }
    }
    /// The surfaces in `scan`. Each foreground item has a colour index that stays with it from scan to scan.
    private(set) var items: [LidarItem] = []
    /// Extra gap in millimetres permitted between neighbours on one surface. Larger values join more points into one item.
    var toleranceMM: CGFloat = 30 {
        didSet { updateItems() }
    }
    @ObservationIgnored private var itemTracker = LidarItemTracker()
    /// Learns from live scans only, so all of the preview scan is foreground.
    @ObservationIgnored private var background = LidarBackground()
    /// Time in seconds when `scan` arrived. The tracker uses it to calculate speeds.
    @ObservationIgnored private var scanTime: TimeInterval = 0
    private(set) var status: Status = .searching

    /// False for Previews: the model shows the given scan and does not open the serial port.
    private let isLive: Bool

    /// Time to wait before the model tries to find or connect to the lidar again.
    private static let retryInterval: Duration = .seconds(2)

    init() {
        scan = []
        isLive = true
    }

    init(previewScan: Lidar360Data) {
        scan = previewScan
        status = .sample
        isLive = false
        updateItems()
    }

    /// Forgets the background and learns it again from the current scan. Use this after the lidar moves.
    func resetBackground() {
        background = LidarBackground()
        background.learn(scan)
        updateItems()
    }

    private func updateItems() {
        items = itemTracker.items(in: scan, toleranceMM: toleranceMM, background: background, time: scanTime)
    }

    /// Xcode Previews and hosted unit tests launch the whole macOS app.
    /// Do not let them take the serial port from the real app.
    private static let isHostedByXcode: Bool = {
        let environment = ProcessInfo.processInfo.environment
        return environment["XCODE_RUNNING_FOR_PREVIEWS"] == "1"
            || environment["XCTestConfigurationFilePath"] != nil
    }()

    /// Reads scans until the calling task is cancelled. Call from a view's `.task`.
    func run() async {
        guard isLive, !Self.isHostedByXcode else { return }

        while !Task.isCancelled {
            if let path = SerialPortFinder.calloutPath() {
                status = .connected(path)
                do {
                    for try await newScan in LidarDevice(path: path).scans() {
                        scan = newScan
                    }
                } catch {
                    status = .error(String(describing: error))
                }
            } else {
                status = .searching
            }
            try? await Task.sleep(for: Self.retryInterval)
        }
    }
}
