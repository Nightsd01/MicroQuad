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
  
  private func parseFloatArray(withData data : Data, floatCount: Int) -> [Float32]?
  {
    let totalBytes = floatCount * MemoryLayout<Float32>.size

    // Ensure the data has enough bytes
    guard data.count >= totalBytes else {
        print("Insufficient data. Expected \(totalBytes) bytes, got \(data.count).")
        return nil
    }

    var floatArray: [Float32] = []
    data.withUnsafeBytes { (pointer: UnsafeRawBufferPointer) in
      var offset = 0
      for _ in 0..<floatCount {
        // Load Float32 value from the current offset
        let float = pointer.load(fromByteOffset: offset, as: Float32.self)
        // Account for endianness if necessary
        let correctedFloat = Float32(bitPattern: UInt32(littleEndian: float.bitPattern))
        floatArray.append(correctedFloat)
        offset += MemoryLayout<Float32>.size
      }
    }
    return floatArray
  }
  
  private func parseXYZ(withData data : Data, type : TelemetryEvent) -> XYZ?
  {
    guard let values = parseFloatArray(withData: data, floatCount: 3) else {
      print("ERROR: Received invalid \(type) update")
      return nil
    }
    return XYZ(x: values[0], y: values[1], z: values[2])
  }
  
  private func parseYawPitchRoll(withData data : Data, type : TelemetryEvent) -> YawPitchRoll?
  {
    guard let values = parseFloatArray(withData: data, floatCount: 3) else {
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
        guard let values = parseFloatArray(withData: payload, floatCount: 1) else {
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
        break
      case .MotorValues:
        guard let values = parseFloatArray(withData: payload, floatCount: 4) else {
          print("ERROR: Received invalid \(type) update")
          return
        }
        self.motorValues = values
        break
      default:
        print("ERROR: Received invalid \(type) update case")
        break
    }
  }
}
