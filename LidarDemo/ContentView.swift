//
//  ContentView.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

struct ContentView: View {
    let nodes = sample360Data
    var body: some View {
        GeometryReader { windowGeometry in
            VStack {
                GeometryReader { plotGeometry in
                    Lidar360Plot(nodes: nodes)
                        .path(in: plotGeometry.frame(in: .local))
                        .offsetBy(dx: plotGeometry.size.width / 2.0, dy: plotGeometry.size.height / 2.0)
                        .stroke()
//                        .frame(width:plotGeometry.size.width/20, height: plotGeometry.size.height*5)
//                        .scaledToFill()
                        .aspectRatio(contentMode: .fit)
                }
                .border(Color.white, width: 1)
//                .padding()
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
        // TEMPORARY: log live scans to the console to check the serial link. Replace in step 6.
        .task {
            guard let path = SerialPortFinder.calloutPath() else {
                print("Lidar not found (USB 0x10c4:0xea60)")
                return
            }
            print("Lidar found on \(path)")
            do {
                var count = 0
                for try await scan in LidarDevice(path: path).scans() {
                    count += 1
                    if count % 10 == 0 {
                        let nearest = scan.map(\.dist).min() ?? 0
                        print("scan \(count): \(scan.count) nodes, nearest \(Int(nearest)) mm")
                    }
                }
            } catch {
                print("Lidar error: \(error)")
            }
        }
   }
}

#Preview {
    ContentView()
}
