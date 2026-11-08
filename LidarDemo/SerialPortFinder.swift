//
//  SerialPortFinder.swift
//  LidarDemo
//

import Foundation
import IOKit
import IOKit.serial

/// Finds the /dev/cu.* path of a USB serial device from its USB vendor and product IDs.
enum SerialPortFinder {
    /// USB IDs of the Silicon Labs CP2102N USB to UART bridge on the RPLIDAR C1.
    static let cp2102nVendorID = 0x10c4
    static let cp2102nProductID = 0xea60

    static func calloutPath(vendorID: Int = cp2102nVendorID, productID: Int = cp2102nProductID) -> String? {
        guard let matching = IOServiceMatching(kIOSerialBSDServiceValue) else { return nil }

        var iterator: io_iterator_t = 0
        guard IOServiceGetMatchingServices(kIOMainPortDefault, matching, &iterator) == KERN_SUCCESS else {
            return nil
        }
        defer { IOObjectRelease(iterator) }

        var service = IOIteratorNext(iterator)
        while service != 0 {
            defer {
                IOObjectRelease(service)
                service = IOIteratorNext(iterator)
            }
            // The USB IDs are on an ancestor of the serial service, so search up through the parents.
            guard intProperty("idVendor", of: service) == vendorID,
                  intProperty("idProduct", of: service) == productID,
                  let path = IORegistryEntryCreateCFProperty(service, kIOCalloutDeviceKey as CFString,
                                                             kCFAllocatorDefault, 0)?
                    .takeRetainedValue() as? String
            else { continue }
            return path
        }
        return nil
    }

    private static func intProperty(_ key: String, of entry: io_registry_entry_t) -> Int? {
        let options = IOOptionBits(kIORegistryIterateRecursively | kIORegistryIterateParents)
        let value = IORegistryEntrySearchCFProperty(entry, kIOServicePlane, key as CFString,
                                                    kCFAllocatorDefault, options)
        return (value as? NSNumber)?.intValue
    }
}
