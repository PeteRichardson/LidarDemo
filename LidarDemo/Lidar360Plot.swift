//
//  Lidar360DistancePlot.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

/// Draws one lidar revolution as a closed outline, with the lidar at the centre of the rect.
struct Lidar360Plot: Shape {
    let nodes: Lidar360Data
    /// Distance in millimetres that maps to the edge of the plot. Nodes farther away are drawn at the edge.
    let rangeMM: CGFloat

    init(nodes: Lidar360Data, rangeMM: CGFloat) {
        self.nodes = nodes
        self.rangeMM = rangeMM
    }

    /// The lidar measures 0° in the direction of its arrow, with angles clockwise when seen from above.
    /// Draw 0° at the top of the plot and 90° at the right. Screen y increases downwards.
    func toPoint(_ node: LidarNode, center: CGPoint, scale: CGFloat) -> CGPoint {
        let dist = min(node.dist, rangeMM) * scale
        let radians = node.angle * Double.pi / 180
        return CGPoint(x: center.x + dist * sin(radians), y: center.y - dist * cos(radians))
    }

    func path(in rect: CGRect) -> Path {
        var path = Path()
        guard let first = nodes.first, rangeMM > 0 else { return path }

        let center = CGPoint(x: rect.midX, y: rect.midY)
        let scale = min(rect.width, rect.height) / 2 / rangeMM

        path.move(to: toPoint(first, center: center, scale: scale))
        for node in nodes.dropFirst() {
            path.addLine(to: toPoint(node, center: center, scale: scale))
        }
        path.closeSubpath()
        return path
    }
}
