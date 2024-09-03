//
//  LidarDataSource.swift
//  LidarDemo
//
//  Created by Peter Richardson on 9/3/24.
//

import Foundation

struct sl_lidar_response_measurement_node_hq {

    let angle_z_q14 : UInt16    // fixed-point angle in z presentation
    let dist_mm_q2 : UInt32     // Distance in millimeter of fixed point values
    let quality : UInt8         // Measurement quality (0 ~ 255)
    let flag : UInt8            // Flags, current only one bit used: SL_LIDAR_RESP_MEASUREMENT_SYNCBIT
    
    // float angle_in_degrees = angle_z_q14 * 90.f / (1 << 14);
    var angle: CGFloat {
        get {
            CGFloat(angle_z_q14) * 90.0 / CGFloat(1 << 14)
        }
    }
    
    // float distance_in_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
    var dist: CGFloat {
        get {
            //CGFloat(dist_mm_q2) / 1000.0 / CGFloat(1 << 2)
            CGFloat(dist_mm_q2) / CGFloat(1 << 2)
        }
    }
    
    init(angle_z_q14: UInt16, dist_mm_q2: UInt32, quality: UInt8, flag: UInt8) {
        self.angle_z_q14 = angle_z_q14
        self.dist_mm_q2 = dist_mm_q2
        self.quality = quality
        self.flag = flag
    }
    
    init(_ angle: CGFloat, _ dist_mm_q2: UInt32, _ quality: UInt8, _ flag: UInt8) {
        self.angle_z_q14 = UInt16(angle * CGFloat(1 << 14) / 90.0)
        self.dist_mm_q2 = dist_mm_q2
        self.quality = quality
        self.flag = flag
    }
}
typealias LidarNode = sl_lidar_response_measurement_node_hq


struct LidarDataSource {
    var nodes : [LidarNode] = [
        LidarNode(0, 250, 47, 0),
        LidarNode(10, 250, 47, 0),
        LidarNode(30, 250, 47, 0),
        LidarNode(30, 250, 47, 0),
        LidarNode(40, 250, 47, 0),
        LidarNode(50, 250, 47, 0),
        LidarNode(60, 250, 47, 0),
        LidarNode(70, 250, 47, 0),
        LidarNode(80, 250, 47, 0),
        LidarNode(90, 250, 47, 0),
        LidarNode(100, 250, 47, 0),
        LidarNode(110, 250, 47, 0),
        LidarNode(120, 250, 47, 0),
        LidarNode(130, 250, 47, 1),
        LidarNode(140, 250, 47, 2),
        LidarNode(150, 250, 47, 3),
        LidarNode(160, 240, 47, 3),
        LidarNode(170, 240, 47, 3),
        LidarNode(180, 240, 47, 3),
        LidarNode(190, 240, 47, 3),
        LidarNode(200, 240, 47, 3),
        LidarNode(210, 240, 47, 3),
        LidarNode(220, 230, 47, 3),
        LidarNode(230, 220, 47, 3),
        LidarNode(240, 230, 47, 3),
        LidarNode(250, 240, 47, 3),
        LidarNode(260, 250, 47, 3),
        LidarNode(270, 250, 47, 3),
        LidarNode(280, 250, 47, 3),
        LidarNode(290, 250, 47, 3),
        LidarNode(300, 250, 47, 3),
        LidarNode(310, 250, 47, 3),
        LidarNode(320, 250, 47, 3),
        LidarNode(330, 250, 47, 3),
        LidarNode(340, 250, 47, 3),
        LidarNode(350, 250, 47, 3),
    ]
}
