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

public class PIDController {
  let controller : BLEController
  var gains : PIDsContainer
  fileprivate var lastSentTime : Date?
  fileprivate var nextTimer : Timer?
  static let userDefaultsKey = "pid_gains"
  var waitingOnTimer = false
  
  init(controller: BLEController) {
    self.controller = controller
    
    guard let loadedGains = PIDController.loadPIDGains() else {
      self.gains = PIDController.defaultGains()
      return
    }
    self.gains = loadedGains
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
    savePIDGains()
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
      nextTimer = Timer(timeInterval: updateRateSeconds, repeats: false) { timer in
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
  
  func savePIDGains() {
    do {
      // Encode gains to Data using JSON
      let data = try JSONEncoder().encode(gains)
      UserDefaults.standard.set(data, forKey: PIDController.userDefaultsKey)
    } catch {
      print("Failed to encode PIDGains: \(error)")
    }
  }
  
  static func loadPIDGains() -> PIDsContainer? {
    guard let data = UserDefaults.standard.data(forKey: PIDController.userDefaultsKey) else {
      return nil
    }
    do {
      // Decode from JSON Data back into PIDGains
      return try JSONDecoder().decode(PIDsContainer.self, from: data)
    } catch {
      print("Failed to decode PIDGains: \(error)")
      return nil
    }
  }
}
