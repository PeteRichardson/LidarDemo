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
    /// - Parameter outward: Extra distance in points, away from the lidar.
    func toPoint(_ node: LidarNode, center: CGPoint, scale: CGFloat, outward: CGFloat = 0) -> CGPoint {
        let dist = min(node.dist, rangeMM) * scale + outward
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

    /// Draws the nodes as one connected line, or as a dot if there is only one node.
    /// - Parameter outward: Moves each point this distance in points away from the lidar.
    func path(through nodes: Lidar360Data, in rect: CGRect, outward: CGFloat = 0) -> Path {
        var path = Path()
        guard !nodes.isEmpty, rangeMM > 0 else { return path }

        let center = CGPoint(x: rect.midX, y: rect.midY)
        let scale = min(rect.width, rect.height) / 2 / rangeMM
        let points = nodes.map { toPoint($0, center: center, scale: scale, outward: outward) }
        if points.count == 1 {
            let r = Self.dotRadius
            path.addEllipse(in: CGRect(x: points[0].x - r, y: points[0].y - r, width: 2 * r, height: 2 * r))
        } else {
            path.addLines(points)
        }
        return path
    }

    func path(in rect: CGRect) -> Path {
        var path = Path()
        let (segments, closed) = Self.segments(nodes, toleranceMM: toleranceMM)
        for segment in segments {
            // For a closed ring, go back to the first node.
            path.addPath(self.path(through: closed ? segment + segment.prefix(1) : segment, in: rect))
        }
        return path
    }

    /// Splits the nodes into runs that are all inside the range or all beyond it.
    /// The line between an inside node and a beyond node is part of the beyond run, so the runs stay connected.
    static func rangeRuns(_ nodes: Lidar360Data, rangeMM: CGFloat) -> [(nodes: Lidar360Data, beyondRange: Bool)] {
        var runs: [(nodes: Lidar360Data, beyondRange: Bool)] = []
        for node in nodes {
            let beyond = node.dist >= rangeMM
            guard let last = runs.last, let previous = last.nodes.last else {
                runs.append(([node], beyond))
                continue
            }
            if last.beyondRange == beyond {
                runs[runs.count - 1].nodes.append(node)
            } else if beyond {
                runs.append(([previous, node], true))
            } else {
                runs[runs.count - 1].nodes.append(node)
                runs.append(([node], false))
            }
        }
        return runs
    }
}

/// One surface in a scan, probably one object.
struct LidarItem {
    /// Nodes in scan order. For a closed ring, the first node is repeated at the end.
    let nodes: Lidar360Data
    /// Identifies the colour of the item. Nil for a single node, which is probably noise.
    let colorIndex: Int?
}

/// Splits each scan into items, and gives an item the same colour index as the nearest item in the previous scan.
/// An item that is hidden and then seen again gets a new colour.
struct LidarItemTracker {
    /// An item that moves farther than this between two scans is a new item.
    static let maxMoveMM: CGFloat = 200

    private var previous: [(centroid: CGPoint, colorIndex: Int)] = []
    private var nextColorIndex = 0

    mutating func items(in scan: Lidar360Data, toleranceMM: CGFloat) -> [LidarItem] {
        let (segments, closed) = Lidar360Plot.segments(scan, toleranceMM: toleranceMM)
        var unmatched = previous
        var current: [(centroid: CGPoint, colorIndex: Int)] = []

        let items = segments.map { segment -> LidarItem in
            let nodes = closed ? segment + segment.prefix(1) : segment
            guard segment.count > 1 else { return LidarItem(nodes: nodes, colorIndex: nil) }

            let centroid = Self.centroid(segment)
            let nearest = unmatched.indices.min { Self.distance(unmatched[$0].centroid, centroid) < Self.distance(unmatched[$1].centroid, centroid) }
            let colorIndex: Int
            if let nearest, Self.distance(unmatched[nearest].centroid, centroid) <= Self.maxMoveMM {
                colorIndex = unmatched.remove(at: nearest).colorIndex
            } else {
                colorIndex = nextColorIndex
                nextColorIndex += 1
            }
            current.append((centroid, colorIndex))
            return LidarItem(nodes: nodes, colorIndex: colorIndex)
        }
        previous = current
        return items
    }

    /// Mean position of the nodes in millimetres, with the lidar at the origin.
    static func centroid(_ nodes: Lidar360Data) -> CGPoint {
        let points = nodes.map { node in
            let radians = node.angle * Double.pi / 180
            return CGPoint(x: node.dist * sin(radians), y: node.dist * cos(radians))
        }
        let count = CGFloat(points.count)
        return CGPoint(x: points.map(\.x).reduce(0, +) / count, y: points.map(\.y).reduce(0, +) / count)
    }

    private static func distance(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
        hypot(a.x - b.x, a.y - b.y)
    }
}

/// Draws each item in its own bright colour. Parts beyond the range are dim.
struct LidarItemsView: View {
    let items: [LidarItem]
    let rangeMM: CGFloat

    static let beyondRangeColor = Color.white.opacity(0.25)
    static let singleNodeColor = Color.gray

    /// Bright colour for an item. The golden-ratio step makes consecutive items far apart in hue.
    static func color(for colorIndex: Int) -> Color {
        let hue = (Double(colorIndex) * 0.618_034).truncatingRemainder(dividingBy: 1)
        return Color(hue: hue, saturation: 0.75, brightness: 1)
    }

    var body: some View {
        Canvas { context, size in
            let plot = Lidar360Plot(nodes: [], rangeMM: rangeMM)
            let rect = CGRect(origin: .zero, size: size)
            for item in items {
                let itemColor = item.colorIndex.map(Self.color(for:)) ?? Self.singleNodeColor
                for run in Lidar360Plot.rangeRuns(item.nodes, rangeMM: rangeMM) {
                    if run.beyondRange {
                        context.stroke(plot.path(through: run.nodes, in: rect), with: .color(Self.beyondRangeColor))
                    } else {
                        context.stroke(plot.path(through: run.nodes, in: rect), with: .color(itemColor))
                        // A second line behind the item, away from the lidar, makes the item thicker.
                        if item.colorIndex != nil {
                            context.stroke(plot.path(through: run.nodes, in: rect, outward: 1), with: .color(itemColor))
                        }
                    }
                }
            }
        }
    }
}
