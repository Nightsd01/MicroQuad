//
//  QuadStatus.swift
//  QuadController
//
//  Created by Brad Hesse on 1/23/21.
//

import Foundation

public struct XYZ {
  let x : Float32
  let y : Float32
  let z : Float32
}

public struct YawPitchRoll {
  let yaw : Float32
  let pitch : Float32
  let roll : Float32
}

public struct MemoryStatus {
  // Represents how much space (in bytes) is currently free
  let freeHeapByteCount : UInt32
  
  // Represents the current lowest/worst amount of heap space since boot
  let minFreeHeapByteCount : UInt32
  
  // Represents the total allocated capacity of the heap
  let totalHeapSizeByteCount : UInt32
}

public class QuadStatus : ObservableObject {
  @Published var rssi : Double?
  @Published var connected = false
  @Published var armed = false
  @Published var motorValues : [Float32]?
  @Published var batteryVoltage : Float32?
  @Published var batteryPercent : Float32?
  @Published var euler : YawPitchRoll?
  @Published var accelRaw : XYZ?
  @Published var accelFiltered : XYZ?
  @Published var gyroRaw : XYZ?
  @Published var gyroFiltered : XYZ?
  @Published var memoryStatus : MemoryStatus?
  @Published var loopUpdateRateHz : UInt64?
  
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
        self.armed = byte == 1 ? true : false
        break
      case .BatteryVoltage:
        guard let values : [Float32] = parseNumericArray(withData: payload, count: 1) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.batteryVoltage = values[0]
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
      default:
        print("ERROR: Received invalid \(type) update case")
        break
    }
  }
}
