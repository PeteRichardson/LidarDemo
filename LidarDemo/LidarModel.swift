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

    private(set) var scan: Lidar360Data
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
