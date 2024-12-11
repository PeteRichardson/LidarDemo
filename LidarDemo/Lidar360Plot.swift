//
//  Lidar360DistancePlot.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

struct Lidar360Plot: Shape {
    let nodes : Lidar360Data
    
    init(nodes: Lidar360Data) {
        self.nodes = nodes
    }
    
    func toPoint(_ node: LidarNode) -> CGPoint {
        let stopX = node.dist * cos(node.angle * Double.pi / 180)
        let stopY = node.dist * sin(node.angle * Double.pi / 180)
        return CGPoint(x: stopX, y: stopY)
    }
    
    func path(in rect: CGRect) -> Path {
        var path = Path()
        
        path.move(to: toPoint(nodes[0]))

        for node in self.nodes {
            let nextPoint = toPoint(node)
            //print("moveTo (\(nextPoint.x),\(nextPoint.y))")
            path.addLine(to: nextPoint)
        }
        path.closeSubpath()
        return path
    }
}
