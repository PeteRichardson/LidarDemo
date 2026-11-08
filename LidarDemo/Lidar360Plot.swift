//
//  Lidar360DistancePlot.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import SwiftUI

/// Draws one lidar revolution, with the lidar at the centre of the rect.
/// Each surface is a separate line, so a near object does not connect to the wall behind it.
struct Lidar360Plot: Shape {
    let nodes: Lidar360Data
    /// Distance in millimetres that maps to the edge of the plot. Nodes farther away are drawn at the edge.
    let rangeMM: CGFloat
    /// Extra gap in millimetres that is permitted between two neighbours on one surface, for sensor noise.
    let toleranceMM: CGFloat

    /// The smallest angle between a surface and the beam at which the surface stays one segment.
    /// A surface that is more nearly parallel to the beam breaks into separate points.
    static let minSurfaceAngle = 10.0 * Double.pi / 180
    /// Radius in points of the dot for a node that has no neighbour on its surface.
    static let dotRadius: CGFloat = 1

    init(nodes: Lidar360Data, rangeMM: CGFloat, toleranceMM: CGFloat = 30) {
        self.nodes = nodes
        self.rangeMM = rangeMM
        self.toleranceMM = toleranceMM
    }

    /// The lidar measures 0° in the direction of its arrow, with angles clockwise when seen from above.
    /// Draw 0° at the top of the plot and 90° at the right. Screen y increases downwards.
    func toPoint(_ node: LidarNode, center: CGPoint, scale: CGFloat) -> CGPoint {
        let dist = min(node.dist, rangeMM) * scale
        let radians = node.angle * Double.pi / 180
        return CGPoint(x: center.x + dist * sin(radians), y: center.y - dist * cos(radians))
    }

    /// Adaptive breakpoint detection (Borges & Aldon, 2004).
    /// Two neighbours are on one surface if the gap between them is not more than the gap on a surface
    /// at `minSurfaceAngle` to the beam, plus `toleranceMM`. The permitted gap increases with range,
    /// because adjacent beams spread apart.
    /// - Parameter step: Angle in radians between the two beams.
    static func isSameSurface(_ a: LidarNode, _ b: LidarNode, step: Double, toleranceMM: CGFloat) -> Bool {
        // A large angle step means missing readings, so the surface between them is unknown.
        guard step < minSurfaceAngle else { return false }
        // Use the nearer range, so that the result does not change with the scan direction.
        let range = min(a.dist, b.dist)
        let maxGap = range * sin(step) / sin(minSurfaceAngle - step) + toleranceMM
        // Straight-line distance between the two points (law of cosines).
        let gap = sqrt(a.dist * a.dist + b.dist * b.dist - 2 * a.dist * b.dist * cos(step))
        return gap <= maxGap
    }

    /// Splits the scan into runs of adjacent nodes that are on one surface.
    /// `closed` is true when the scan is one surface all the way around.
    static func segments(_ nodes: Lidar360Data, toleranceMM: CGFloat) -> (segments: [Lidar360Data], closed: Bool) {
        var segments: [Lidar360Data] = []
        for node in nodes {
            if let previous = segments.last?.last,
               isSameSurface(previous, node, step: abs(node.angle - previous.angle) * Double.pi / 180, toleranceMM: toleranceMM) {
                segments[segments.count - 1].append(node)
            } else {
                segments.append([node])
            }
        }

        // Join the last node to the first node across 360°, if they are on one surface.
        guard nodes.count > 1, let first = nodes.first, let last = nodes.last else { return (segments, false) }
        let wrapStep = (first.angle + 360 - last.angle) * Double.pi / 180
        guard isSameSurface(last, first, step: wrapStep, toleranceMM: toleranceMM) else { return (segments, false) }
        if segments.count == 1 {
            return (segments, true)
        }
        segments[0] = segments.removeLast() + segments[0]
        return (segments, false)
    }

    func path(in rect: CGRect) -> Path {
        var path = Path()
        guard !nodes.isEmpty, rangeMM > 0 else { return path }

        let center = CGPoint(x: rect.midX, y: rect.midY)
        let scale = min(rect.width, rect.height) / 2 / rangeMM

        let (segments, closed) = Self.segments(nodes, toleranceMM: toleranceMM)
        for segment in segments {
            let points = segment.map { toPoint($0, center: center, scale: scale) }
            if points.count == 1 {
                let r = Self.dotRadius
                path.addEllipse(in: CGRect(x: points[0].x - r, y: points[0].y - r, width: 2 * r, height: 2 * r))
            } else {
                path.addLines(points)
            }
        }
        if closed {
            path.closeSubpath()
        }
        return path
    }
}
