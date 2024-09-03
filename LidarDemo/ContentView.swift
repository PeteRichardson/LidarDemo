//
//  ContentView.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

struct ContentView: View {
    var body: some View {
        GeometryReader { windowGeometry in
            VStack {
                GeometryReader { plotGeometry in
                    Lidar360DistancePlot()
                        .path(in: plotGeometry.frame(in: .local))
                        .stroke()
                        .scaledToFit()
                }
                .padding()
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
   }
}

#Preview {
    ContentView()
}
