//
//  RectangleFit.swift
//  LidarDemo
//

import Foundation

/// Fits the known rectangular cross section of the object to the lidar points on its surface.
/// The lidar sees only the one or two faces that point at it, so the fit puts those faces on the points
/// and puts the rest of the rectangle behind them.
enum RectangleFit {
    static let lengthMM: CGFloat = 59
    static let widthMM: CGFloat = 29
    /// Points of a larger item are not all within this distance of their mean, so the item is not the object.
    /// The value is half of the diagonal, plus an allowance for the range noise.
    static let maxRadiusMM: CGFloat = hypot(lengthMM, widthMM) / 2 + 30
    /// Number of orientations to try, evenly spaced over 180°. The rectangle is the same after a 180° turn.
    static let angleSteps = 180

    /// Position of the rectangle, in millimetres with the lidar at the origin.
    struct Pose: Equatable {
        var center: CGPoint
        /// Direction of the long side in radians, from 0 up to but not including π.
        var angle: Double

        /// Unit vectors along the long side and along the short side.
        var axes: (long: CGVector, short: CGVector) {
            (CGVector(dx: cos(angle), dy: sin(angle)), CGVector(dx: -sin(angle), dy: cos(angle)))
        }

        var corners: [CGPoint] {
            let (u, v) = axes
            let a = RectangleFit.lengthMM / 2
            let b = RectangleFit.widthMM / 2
            return [(a, b), (-a, b), (-a, -b), (a, -b)].map { su, sv in
                CGPoint(x: center.x + su * u.dx + sv * v.dx, y: center.y + su * u.dy + sv * v.dy)
            }
        }

        /// Moves part of the way from the predicted pose to the measured pose.
        /// - Parameters:
        ///   - shift: Predicted movement of the centre since this pose.
        ///   - weight: Fraction of the difference to the measured pose. 1 gives the measured pose.
        func smoothed(toward measured: Pose, shift: CGVector, weight: CGFloat) -> Pose {
            let predicted = CGPoint(x: center.x + shift.dx, y: center.y + shift.dy)
            let newCenter = CGPoint(x: predicted.x + weight * (measured.center.x - predicted.x),
                                    y: predicted.y + weight * (measured.center.y - predicted.y))
            // The shortest turn to the measured angle, because the angle repeats every π.
            let turn = remainder(measured.angle - angle, .pi)
            return Pose(center: newCenter, angle: Self.normalized(angle + Double(weight) * turn))
        }

        static func normalized(_ angle: Double) -> Double {
            let a = angle.truncatingRemainder(dividingBy: .pi)
            return a < 0 ? a + .pi : a
        }
    }

    /// The pose with the smallest error, or nil if the points cannot be the object.
    static func fit(_ points: [CGPoint]) -> Pose? {
        guard points.count >= 2 else { return nil }
        let count = CGFloat(points.count)
        let mean = CGPoint(x: points.map(\.x).reduce(0, +) / count, y: points.map(\.y).reduce(0, +) / count)
        guard points.allSatisfy({ hypot($0.x - mean.x, $0.y - mean.y) <= maxRadiusMM }) else { return nil }

        var best: (pose: Pose, error: CGFloat)?
        for step in 0..<angleSteps {
            let pose = place(points, angle: Double(step) * .pi / Double(angleSteps))
            let error = squaredError(points, pose)
            if best.map({ error < $0.error }) ?? true {
                best = (pose, error)
            }
        }
        return best?.pose
    }

    /// Puts the rectangle at the given angle so that the faces on the lidar side touch the nearest points.
    static func place(_ points: [CGPoint], angle: Double) -> Pose {
        let (u, v) = Pose(center: .zero, angle: angle).axes
        let alongLong = points.map { $0.x * u.dx + $0.y * u.dy }
        let alongShort = points.map { $0.x * v.dx + $0.y * v.dy }
        let cu = centerCoordinate(alongLong, halfSize: lengthMM / 2)
        let cv = centerCoordinate(alongShort, halfSize: widthMM / 2)
        return Pose(center: CGPoint(x: cu * u.dx + cv * v.dx, y: cu * u.dy + cv * v.dy), angle: angle)
    }

    /// Centre of the rectangle on one axis. The lidar is at 0 on each axis.
    /// If the lidar is below all the points, it sees the low face, so the centre is half a size above the lowest point.
    /// If the lidar is between the points, it sees no face across this axis, so the centre is in the middle of the points.
    private static func centerCoordinate(_ values: [CGFloat], halfSize: CGFloat) -> CGFloat {
        guard let low = values.min(), let high = values.max() else { return 0 }
        if low > 0 { return low + halfSize }
        if high < 0 { return high - halfSize }
        return (low + high) / 2
    }

    /// Sum of the squared distances from the points to the edges of the rectangle, plus `shadowError`.
    static func squaredError(_ points: [CGPoint], _ pose: Pose) -> CGFloat {
        let (u, v) = pose.axes
        let a = lengthMM / 2
        let b = widthMM / 2
        return shadowError(points, pose) + points.reduce(0) { sum, point in
            let dx = point.x - pose.center.x
            let dy = point.y - pose.center.y
            let qu = abs(dx * u.dx + dy * u.dy)
            let qv = abs(dx * v.dx + dy * v.dy)
            let distance = qu <= a && qv <= b
                ? min(a - qu, b - qv)
                : hypot(max(qu - a, 0), max(qv - b, 0))
            return sum + distance * distance
        }
    }

    /// Error for the part of the rectangle that the lidar must see but that has no points.
    /// Without this, a short face seen straight on also fits as a part of a long face.
    static func shadowError(_ points: [CGPoint], _ pose: Pose) -> CGFloat {
        guard points.count >= 2 else { return 0 }
        // Bearings relative to the rectangle's centre, so that the values do not wrap at ±π.
        let reference = atan2(pose.center.y, pose.center.x)
        func bearing(_ p: CGPoint) -> Double { remainder(atan2(p.y, p.x) - reference, 2 * .pi) }

        let pointBearings = points.map(bearing)
        let cornerBearings = pose.corners.map(bearing)
        guard let pointLow = pointBearings.min(), let pointHigh = pointBearings.max(),
              let cornerLow = cornerBearings.min(), let cornerHigh = cornerBearings.max() else { return 0 }

        // A beam can miss the end of a face by up to one step between beams, so permit that much.
        let step = (pointHigh - pointLow) / Double(points.count - 1)
        let excess = max(pointLow - cornerLow - step, 0) + max(cornerHigh - pointHigh - step, 0)
        let excessMM = CGFloat(excess) * hypot(pose.center.x, pose.center.y)
        return excessMM * excessMM * CGFloat(points.count)
    }
}
