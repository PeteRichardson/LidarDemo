//
//  Lidar360DistancePlot.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

struct Lidar360DistancePlot: Shape {
    func path(in rect: CGRect) -> Path {
        // temp code to draw an interesting shape copied from BaronSharktooth
        // in https://www.reddit.com/r/SwiftUI/comments/onyhyb/how_would_i_create_a_circle_like_this_by_swiftui/
        var path = Path()
        let radius = min(rect.maxX - rect.midX, rect.maxY - rect.midY)
        let start: CGFloat = 0
        let stop: CGFloat = CGFloat.pi * 2.0
        let step: CGFloat = CGFloat.pi / 18
        path.move(to: CGPoint(x: rect.midX + radius, y: rect.midY))

        for angle in stride(from: start, to: stop, by: step) {
            let stopX = rect.midX + radius * cos(angle)
            let stopY = rect.midY + radius * sin(angle)
            let endPoint = CGPoint(x: stopX, y: stopY)
            
            let randomX = rect.midX + (radius * 1.1) * cos(angle)
            let randomY = rect.midY + (radius * 1.1) * sin(angle)
            let randomPoint = CGPoint(x: randomX, y: randomY)

            path.addQuadCurve(to: endPoint, control: randomPoint)
        }
        path.closeSubpath()
        return path
    }
}
