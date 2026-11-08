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

    /// Converts a position in millimetres, with y in the direction of the lidar's arrow, to a point in the rect.
    /// Unlike `toPoint(_:center:scale:outward:)`, a position beyond the range is not moved to the edge.
    func toPoint(mm point: CGPoint, in rect: CGRect) -> CGPoint {
        let scale = min(rect.width, rect.height) / 2 / rangeMM
        return CGPoint(x: rect.midX + point.x * scale, y: rect.midY - point.y * scale)
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
    /// Position and speed of a tracked item, in millimetres with the lidar at the origin.
    /// x is to the right of the lidar's arrow, and y is in the direction of the arrow.
    struct Motion {
        /// Centre of the fitted rectangle if there is one, otherwise the mean of the nodes.
        let centroid: CGPoint
        /// Millimetres per second.
        let velocity: CGVector

        var speed: CGFloat { hypot(velocity.dx, velocity.dy) }
    }

    /// Nodes in scan order. For a closed ring, the first node is repeated at the end.
    let nodes: Lidar360Data
    /// Identifies the colour of the item. Nil for background, and for a single node, which is probably noise.
    let colorIndex: Int?
    /// Nil if the item is not tracked.
    var motion: Motion? = nil
    /// The known cross section of the object, fitted to the nodes. Nil if the item is not tracked, or is too large to be the object.
    var rectangle: RectangleFit.Pose? = nil
}

/// Splits each scan into items, and gives a foreground item the same colour index as the nearest item in the previous scan.
/// Also calculates the speed of each item. An item that is hidden and then seen again gets a new colour.
struct LidarItemTracker {
    /// An item that is farther than this from its predicted position is a new item.
    static let maxMoveMM: CGFloat = 200
    /// An item is foreground if at least this fraction of its nodes are foreground.
    static let minForegroundFraction = 0.5
    /// Weight of the newest measured velocity. The centroid moves when the ends of an item change,
    /// so a smaller weight gives a smoother but slower speed.
    static let velocitySmoothing: CGFloat = 0.3
    /// Weight of the newest rectangle fit. The range noise is as large as the object, so one fit is not accurate.
    static let rectangleSmoothing: CGFloat = 0.3

    private struct Track {
        /// Centre of the rectangle if there is one, otherwise the mean of the nodes.
        let centroid: CGPoint
        let colorIndex: Int
        let velocity: CGVector
        let rectangle: RectangleFit.Pose?
    }

    private var previous: [Track] = []
    private var previousTime: TimeInterval?
    private var nextColorIndex = 0

    /// - Parameters:
    ///   - background: The default has no samples, so all items are foreground.
    ///   - time: Time of the scan in seconds. Use the same time again for the same scan, for example after a change to `toleranceMM`.
    mutating func items(
        in scan: Lidar360Data, toleranceMM: CGFloat, background: LidarBackground = LidarBackground(), time: TimeInterval = 0
    ) -> [LidarItem] {
        let (segments, closed) = Lidar360Plot.segments(scan, toleranceMM: toleranceMM)
        let dt = previousTime.map { time - $0 } ?? 0
        // Where each previous item is probably now, if its speed did not change.
        let predicted = previous.map { CGPoint(x: $0.centroid.x + $0.velocity.dx * dt, y: $0.centroid.y + $0.velocity.dy * dt) }
        var unmatched = Array(previous.indices)
        var current: [Track] = []

        let items = segments.map { segment -> LidarItem in
            let nodes = closed ? segment + segment.prefix(1) : segment
            let foregroundCount = segment.count(where: background.isForeground)
            guard segment.count > 1, Double(foregroundCount) >= Self.minForegroundFraction * Double(segment.count) else {
                return LidarItem(nodes: nodes, colorIndex: nil)
            }

            let points = segment.map(Self.position)
            let centroid = Self.centroid(points)
            let nearest = unmatched.indices.min { Self.distance(predicted[unmatched[$0]], centroid) < Self.distance(predicted[unmatched[$1]], centroid) }
            let track: Track
            if let nearest, Self.distance(predicted[unmatched[nearest]], centroid) <= Self.maxMoveMM {
                let old = previous[unmatched.remove(at: nearest)]
                let rectangle = RectangleFit.fit(points).map { measured in
                    old.rectangle.map {
                        $0.smoothed(toward: measured, shift: CGVector(dx: old.velocity.dx * dt, dy: old.velocity.dy * dt), weight: Self.rectangleSmoothing)
                    } ?? measured
                }
                let position = rectangle?.center ?? centroid
                track = Track(centroid: position, colorIndex: old.colorIndex, velocity: Self.smoothed(old, to: position, dt: dt), rectangle: rectangle)
            } else {
                let rectangle = RectangleFit.fit(points)
                track = Track(centroid: rectangle?.center ?? centroid, colorIndex: nextColorIndex, velocity: .zero, rectangle: rectangle)
                nextColorIndex += 1
            }
            current.append(track)
            return LidarItem(nodes: nodes, colorIndex: track.colorIndex,
                             motion: .init(centroid: track.centroid, velocity: track.velocity), rectangle: track.rectangle)
        }
        previous = current
        previousTime = time
        return items
    }

    /// Mixes the velocity measured since the previous scan into the previous velocity.
    private static func smoothed(_ old: Track, to centroid: CGPoint, dt: TimeInterval) -> CGVector {
        guard dt > 0 else { return old.velocity }
        let measured = CGVector(dx: (centroid.x - old.centroid.x) / dt, dy: (centroid.y - old.centroid.y) / dt)
        let k = velocitySmoothing
        return CGVector(dx: k * measured.dx + (1 - k) * old.velocity.dx, dy: k * measured.dy + (1 - k) * old.velocity.dy)
    }

    /// Position of the node in millimetres, with the lidar at the origin and y in the direction of the lidar's arrow.
    static func position(_ node: LidarNode) -> CGPoint {
        let radians = node.angle * Double.pi / 180
        return CGPoint(x: node.dist * sin(radians), y: node.dist * cos(radians))
    }

    static func centroid(_ points: [CGPoint]) -> CGPoint {
        let count = CGFloat(points.count)
        return CGPoint(x: points.map(\.x).reduce(0, +) / count, y: points.map(\.y).reduce(0, +) / count)
    }

    private static func distance(_ a: CGPoint, _ b: CGPoint) -> CGFloat {
        hypot(a.x - b.x, a.y - b.y)
    }
}

/// Draws each foreground item in its own bright colour, with an arrow that shows its speed and direction.
/// Background and parts beyond the range are dim.
struct LidarItemsView: View {
    let items: [LidarItem]
    let rangeMM: CGFloat

    static let beyondRangeColor = Color.white.opacity(0.25)
    static let backgroundColor = Color.white.opacity(0.4)
    /// The arrow points to where the item will be after this time, in seconds.
    static let arrowSeconds: CGFloat = 0.5
    /// Items slower than this, in millimetres per second, have no arrow. This hides the movement that noise causes.
    static let minArrowSpeed: CGFloat = 100
    /// Length in points of each side of the arrow head.
    static let arrowHeadLength: CGFloat = 8
    /// The rectangle is a little dimmer than the item's nodes, so that you can see if the fit is wrong.
    static let rectangleOpacity = 0.55

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
                let itemColor = item.colorIndex.map(Self.color(for:)) ?? Self.backgroundColor
                if let rectangle = item.rectangle {
                    var path = Path()
                    path.addLines(rectangle.corners.map { plot.toPoint(mm: $0, in: rect) })
                    path.closeSubpath()
                    context.stroke(path, with: .color(itemColor.opacity(Self.rectangleOpacity)), lineWidth: 1.5)
                }
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
                if let motion = item.motion {
                    drawArrow(motion, color: itemColor, plot: plot, rect: rect, in: &context)
                }
            }
        }
    }

    private func drawArrow(_ motion: LidarItem.Motion, color: Color, plot: Lidar360Plot, rect: CGRect, in context: inout GraphicsContext) {
        guard motion.speed >= Self.minArrowSpeed, hypot(motion.centroid.x, motion.centroid.y) < rangeMM else { return }

        let tipMM = CGPoint(x: motion.centroid.x + motion.velocity.dx * Self.arrowSeconds,
                            y: motion.centroid.y + motion.velocity.dy * Self.arrowSeconds)
        let start = plot.toPoint(mm: motion.centroid, in: rect)
        let tip = plot.toPoint(mm: tipMM, in: rect)

        // Two short lines back from the tip, at 25° to each side of the shaft.
        let back = atan2(start.y - tip.y, start.x - tip.x)
        let side = 25 * Double.pi / 180
        var path = Path()
        path.move(to: start)
        path.addLine(to: tip)
        for angle in [back - side, back + side] {
            path.move(to: tip)
            path.addLine(to: CGPoint(x: tip.x + Self.arrowHeadLength * cos(angle), y: tip.y + Self.arrowHeadLength * sin(angle)))
        }
        context.stroke(path, with: .color(color), lineWidth: 1.5)

        let label = Text(String(format: "%.2f m/s", motion.speed / 1000)).font(.caption2).foregroundStyle(color)
        // Put the label past the tip, so that it does not cover the arrow.
        let labelPoint = CGPoint(x: tip.x - 14 * cos(back), y: tip.y - 14 * sin(back))
        context.draw(label, at: labelPoint)
    }
}
