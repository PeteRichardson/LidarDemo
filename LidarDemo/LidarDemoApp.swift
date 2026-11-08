//
//  LidarDemoApp.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

@main
struct LidarDemoApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
        }
        // Square, because the plot is a circle.
        .defaultSize(width: 700, height: 700)
    }
}
