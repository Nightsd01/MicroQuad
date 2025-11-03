//
//  PIDControlller.swift
//  QuadController
//
//  Created by Brad Hesse on 3/18/25.
//

import Foundation

struct PIDValues : Codable {
  var proportional : Float32
  var integral : Float32
  var derivative : Float32
}

struct PIDGains : Codable {
  var yawGains : PIDValues
  var pitchGains : PIDValues
  var rollGains : PIDValues
}

struct PIDsContainer : Codable {
  var angleValues : PIDGains
  var rateValues : PIDGains
  var verticalGains : PIDValues
}

fileprivate struct PIDUpdateParams {
  let axis : ControlAxis
  let angleUpdate : PIDGains
  let rateUpdate : PIDGains
}

public class PIDController: ObservableObject {
  weak var controller : BLEController?
  @Published var gains : PIDsContainer
  fileprivate var lastSentTime : Date?
  fileprivate var nextTimer : Timer?
  var waitingOnTimer = false
  
  init(controller: BLEController) {
    self.controller = controller
    
    // Check if we already received PID configuration from ESP32
    if let storedConfig = controller.lastReceivedPIDConfig {
      self.gains = storedConfig
      print("PIDController: Using previously received PID configuration")
    } else {
      // Initialize with default gains, will be updated when ESP32 sends configuration
      self.gains = PIDController.defaultGains()
      print("PIDController: Using default gains, waiting for ESP32 configuration")
    }
  }
  
  private static func defaultGains() -> PIDsContainer
  {
    return PIDsContainer(
      angleValues: PIDGains(
        yawGains: PIDValues(
          proportional: 2.0,
          integral: 0.01,
          derivative: 0.0
        ),
        pitchGains: PIDValues(
          proportional: 3.0,
          integral: 0.02,
          derivative: 0.01
        ),
        rollGains: PIDValues(
          proportional: 3.0,
          integral: 0.02,
          derivative: 0.01
        )
      ),
      rateValues: PIDGains(
        yawGains: PIDValues(
          proportional: 2.1,
          integral: 0.01,
          derivative: 0.0005
        ),
        pitchGains: PIDValues(
          proportional: 4.0,
          integral: 0.02,
          derivative: 0.01
        ),
        rollGains: PIDValues(
          proportional: 4.0,
          integral: 0.02,
          derivative: 0.01
        )
      ),
      verticalGains: PIDValues(
        proportional: 8.0,
        integral: 0.1,
        derivative: 2.0
      )
    )
  }
  
  private func sendUpdate(axis: ControlAxis, type: PIDType) {
    guard let controller = controller else {
      print("PIDController: BLE controller has been deallocated")
      return
    }
    lastSentTime = Date()
    switch (type) {
      case .Angle:
        controller.updatePID(axis: axis, type: type, gains: self.gains.angleValues)
        break
      case .Rate:
        controller.updatePID(axis: axis, type: type, gains: self.gains.rateValues)
        break
      case .VerticalVelocity:
        controller.updatePIDValues(axis: axis, type: type, values: &self.gains.verticalGains)
        break
      default:
        fatalError("Invalid PID type \(type)")
    }
  }
  
  let updateRateSeconds = 0.1
  func updatePids(axis: ControlAxis, type: PIDType, newGains: PIDGains)
  {
    switch (type) {
      case .Angle:
        gains.angleValues = newGains
        break
      case .Rate:
        gains.rateValues = newGains
        break
      default:
        fatalError("Invalid PID type \(type)")
    }
    scheduleUpdateTransmission(axis: axis, type: type)
  }
  
  func updatePidValues(axis: ControlAxis, type : PIDType, newValues : PIDValues)
  {
    switch (type) {
    case .VerticalVelocity:
      gains.verticalGains = newValues
      break
    default:
      fatalError("Invalid PID type \(type) - use updatePids() instead")
    }
    scheduleUpdateTransmission(axis: axis, type: type)
  }
  
  func scheduleUpdateTransmission(axis: ControlAxis, type : PIDType)
  {
    if let lastSentTimeSeconds = self.lastSentTime?.timeIntervalSince1970 {
      if Date().timeIntervalSince1970 - lastSentTimeSeconds > updateRateSeconds {
        self.sendUpdate(axis: axis, type: type)
        return
      } else if waitingOnTimer {
        return
      }
      waitingOnTimer = true
      nextTimer?.invalidate()
      nextTimer = Timer(timeInterval: updateRateSeconds, repeats: false) { timexr in
        self.sendUpdate(axis: axis, type: type)
        self.waitingOnTimer = false
      }
      return
    }
    self.sendUpdate(axis: axis, type: type)
  }
  
  func resetToDefaults() -> PIDsContainer
  {
    self.gains = PIDController.defaultGains()
    return self.gains
  }
  
  
}
