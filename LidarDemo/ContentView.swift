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
    /// Extra gap in millimetres permitted between neighbours on one surface. Larger values join more points into one shape.
    @State private var toleranceMM: CGFloat = 30

    init(model: LidarModel? = nil) {
        _model = State(initialValue: model ?? LidarModel())
    }

    var body: some View {
        GeometryReader { windowGeometry in
            VStack {
                Lidar360Plot(nodes: model.scan, rangeMM: rangeMM, toleranceMM: toleranceMM)
                    .stroke()
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
                    Slider(value: $toleranceMM, in: 0...200)
                    Text(String(format: "%.0f mm", toleranceMM))
                        .monospacedDigit()
                }
                Text(statusText)
                    .font(.caption)
                Text("Lidar Data")
                    .textScale(.default)
                    .font(.largeTitle)
                    .font(.system(size: min(windowGeometry.size.width, windowGeometry.size.height) * 0.075))
                    .padding()
            }
            .padding()
            .foregroundColor(Color.white)
            .background(Color.black)
        }
        .task {
            await model.run()
        }
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
