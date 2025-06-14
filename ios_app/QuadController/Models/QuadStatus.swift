//
//  QuadStatus.swift
//  QuadController
//
//  Created by Brad Hesse on 1/23/21.
//

import Foundation

public struct MemoryStatus {
  // Represents how much space (in bytes) is currently free
  let freeHeapByteCount : UInt32
  
  // Represents the current lowest/worst amount of heap space since boot
  let minFreeHeapByteCount : UInt32
  
  // Represents the total allocated capacity of the heap
  let totalHeapSizeByteCount : UInt32
}

// GPS telemetry data structure matching gps_telem_event_t
public struct GPSData {
  let latitude: Float32       // 4 bytes
  let longitude: Float32      // 4 bytes
  let altitude: Float32       // 4 bytes
  let hdop: Float32          // 4 bytes
  let satellites: UInt8      // 1 byte
  let fixQuality: UInt8      // 1 byte
}

// Battery status event structure matching battery_status_event_t
public struct BatteryStatusData {
  let voltage: Float32        // 4 bytes
  let status: BatteryStatus   // 1 byte (UInt8)
}

public class QuadStatus : ObservableObject {
  @Published var rssi : Double?
  @Published var connected = false
  @Published var armed = false
  @Published var motorValues : [Float32]?
  @Published var batteryVoltage : Float32?
  @Published var batteryPercent : Float32?
  @Published var batteryStatus : BatteryStatus?
  @Published var euler : YawPitchRoll?
  @Published var accelRaw : XYZ?
  @Published var accelFiltered : XYZ?
  @Published var gyroRaw : XYZ?
  @Published var gyroFiltered : XYZ?
  @Published var magRaw : MagRawValues?
  @Published var altitudeEstimate : Double?
  @Published var verticalVelocityEstimate : Double?
  @Published var rangefinderRawDistanceEstimate : Double?
  @Published var rangefinderAltitudeEstimate : Double?
  @Published var memoryStatus : MemoryStatus?
  @Published var loopUpdateRateHz : UInt64?
  @Published var imuUpdateRateHz : UInt64?
  @Published var gpsData : GPSData?
  
  public weak var armStatusListener : ArmStatusListener?
  
  private func parseNumericArray<T : Numeric>(withData data : Data, count: Int) -> [T]?
  {
    let totalBytes = count * MemoryLayout<T>.size

    // Ensure the data has enough bytes
    guard data.count >= totalBytes else {
        print("Insufficient data. Expected \(totalBytes) bytes, got \(data.count).")
        return nil
    }

    var numArray: [T] = []
    data.withUnsafeBytes { (pointer: UnsafeRawBufferPointer) in
      var offset = 0
      for _ in 0..<count {
        // Load T value from the current offset
        let value = pointer.load(fromByteOffset: offset, as: T.self)
        numArray.append(value)
        offset += MemoryLayout<T>.size
      }
    }
    return numArray
  }
  
  private func parseXYZ(withData data : Data, type : TelemetryEvent) -> XYZ?
  {
    guard let values : [Float32] = parseNumericArray(withData: data, count: 3) else {
      print("ERROR: Received invalid \(type) update")
      return nil
    }
    return XYZ(x: values[0], y: values[1], z: values[2])
  }
  
  private func parseYawPitchRoll(withData data : Data, type : TelemetryEvent) -> YawPitchRoll?
  {
    guard let values : [Float32] = parseNumericArray(withData: data, count: 3) else {
      print("ERROR: Received invalid \(type) update")
      return nil
    }
    return YawPitchRoll(yaw: values[0], pitch: values[1], roll: values[2])
  }

  public func updateTelemetry(withData data : Data) {
    // Ensure the data is not empty
    guard !data.isEmpty else {
        print("Received empty data.")
        return
    }
    
    // Extract the first byte
    let firstByte: UInt8 = data[0]
    
    guard let type = TelemetryEvent(rawValue: firstByte) else {
      print("ERROR: Received invalid telemetry packet")
      return
    }
    
    let payload = data.advanced(by: 1) // Skip the first byte
    
    switch (type) {
      case .ArmStatusChange:
        guard !payload.isEmpty else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        let byte = payload[0]
        let armUpdate = byte == 1 ? true : false
        if (self.armed != armUpdate) {
          // we don't want to set sendUpdateToQuadcopter to true as that would
          // result in an infinite loop
          armStatusListener?.handleArmStatusChange(sendUpdateToQuadcopter: false)
        }
        self.armed = armUpdate
        break
      case .BatteryStatusUpdate:
        // Parse battery status structure (float voltage + uint8 status)
        guard payload.count >= 5 else {
          print("ERROR: Received invalid \(type) update - insufficient data")
          return
        }
        
        payload.withUnsafeBytes { (pointer: UnsafeRawBufferPointer) in
          let voltage = pointer.load(fromByteOffset: 0, as: Float32.self)
          let statusRaw = pointer.load(fromByteOffset: 4, as: UInt8.self)
          
          self.batteryVoltage = voltage
          self.batteryStatus = BatteryStatus(rawValue: statusRaw)
          
          // Calculate battery percentage based on voltage
          // Assuming 3.0V = 0%, 4.2V = 100% for a typical LiPo cell
          let minVoltage: Float32 = 3.0
          let maxVoltage: Float32 = 4.2
          let percentage = (voltage - minVoltage) / (maxVoltage - minVoltage)
          self.batteryPercent = min(max(percentage, 0.0), 1.0)
        }
        break
      case .EulerYawPitchRoll:
        self.euler = parseYawPitchRoll(withData: payload, type: type)
        break
      case .AccelerometerXYZRaw:
        self.accelRaw = parseXYZ(withData: payload, type: type)
        break
      case .AccelerometerXYZFiltered:
        self.accelFiltered = parseXYZ(withData: payload, type: type)
        break
      case .GyroXYZRaw:
        self.gyroRaw = parseXYZ(withData: payload, type: type)
        break
      case .GyroXYZFiltered:
        self.gyroFiltered = parseXYZ(withData: payload, type: type)
        break
      case .MemoryStats:
        guard let values : [UInt32] = parseNumericArray(withData: payload, count: 3) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.memoryStatus = MemoryStatus(freeHeapByteCount: values[0], minFreeHeapByteCount: values[1], totalHeapSizeByteCount: values[2])
        break
      case .MotorValues:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 4) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.motorValues = values
        break
      case .LoopUpdateRate:
        guard let values : [UInt64] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.loopUpdateRateHz = values[0]
        break
      case .IMUUpdateRate:
        guard let values : [UInt64] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.imuUpdateRateHz = values[0]
        break
      case .MagnetometerXYZRaw:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 4) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.magRaw = MagRawValues(xyz: XYZ(x: values[0], y: values[1], z: values[2]), heading: values[3])
        break
      case .VL53L1XRawDistance:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.rangefinderRawDistanceEstimate = Double(values[0])
        break
      case .VL53L1XEstimatedAltitudeUpdate:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.rangefinderAltitudeEstimate = Double(values[0])
        break
      case .EKFAltitudeEstimate:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.altitudeEstimate = Double(values[0])
        break
      case .EKFVerticalVelocityEstimate:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.verticalVelocityEstimate = Double(values[0])
        break
      case .GPSFixData:
        // Parse GPS data structure with mixed types
        guard payload.count >= 18 else {
          print("ERROR: Received invalid \(type) update - insufficient data")
          return
        }
        
        payload.withUnsafeBytes { (pointer: UnsafeRawBufferPointer) in
          let latitude = pointer.load(fromByteOffset: 0, as: Float32.self)
          let longitude = pointer.load(fromByteOffset: 4, as: Float32.self)
          let altitude = pointer.load(fromByteOffset: 8, as: Float32.self)
          let hdop = pointer.load(fromByteOffset: 12, as: Float32.self)
          let satellites = pointer.load(fromByteOffset: 16, as: UInt8.self)
          let fixQuality = pointer.load(fromByteOffset: 17, as: UInt8.self)
          
          self.gpsData = GPSData(
            latitude: latitude,
            longitude: longitude,
            altitude: altitude,
            hdop: hdop,
            satellites: satellites,
            fixQuality: fixQuality
          )
        }
        break
      default:
        print("ERROR: Received invalid \(type) update case")
        break
    }
  }
}
