//
//  ContentView.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

struct ContentView: View {
    @State private var model: LidarModel
    /// Distance in millimetres at the edge of the plot. Fixed, so an object that comes near does not change the zoom.
    @State private var rangeMM: CGFloat = 500

    init(model: LidarModel? = nil) {
        _model = State(initialValue: model ?? LidarModel())
    }

    var body: some View {
        VStack {
            LidarItemsView(items: model.items, rangeMM: rangeMM)
                .aspectRatio(1, contentMode: .fit)
                .frame(maxWidth: .infinity, maxHeight: .infinity)
                .border(Color.white, width: 1)
            HStack {
                Text("Range")
                Slider(value: $rangeMM, in: 500...12000)
                Text(String(format: "%.1f m", rangeMM / 1000))
                    .monospacedDigit()
            }
            HStack {
                Text("Gap tolerance")
                Slider(value: $model.toleranceMM, in: 0...200)
                Text(String(format: "%.0f mm", model.toleranceMM))
                    .monospacedDigit()
            }
            HStack {
                Text(statusText)
                    .font(.caption)
                Button("Reset background") {
                    model.resetBackground()
                }
            }
            deviceInfoView
        }
        .padding()
        .frame(maxWidth: .infinity, maxHeight: .infinity)
        .foregroundColor(Color.white)
        .background(Color.black)
        .task {
            await model.run()
        }
    }

    /// Make, model, firmware, hardware and serial number of the lidar, on one small line.
    private var deviceInfoView: some View {
        let make = RPLidarDeviceInfo.make
        let text = model.deviceInfo.map {
            "\(make) \($0.modelText) · Firmware \($0.firmwareText) · Hardware \($0.hardware) · S/N \($0.serialNumberText)"
        } ?? "\(make) · No device info yet"
        return Text(text)
            .font(.caption2)
            .foregroundStyle(.secondary)
            .lineLimit(1)
            .minimumScaleFactor(0.7)
    }

    private var statusText: String {
        switch model.status {
        case .sample: "Sample data (Preview)"
        case .searching: "Lidar not connected (USB 0x10c4:0xea60)"
        case .connected(let path): "Lidar on \(path) — \(model.scan.count) points"
        case .error(let message): "Lidar error: \(message)"
        }
    }
}

#Preview {
    ContentView(model: LidarModel(previewScan: sample360Data))
}
