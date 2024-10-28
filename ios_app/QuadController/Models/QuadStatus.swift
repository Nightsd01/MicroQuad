//
//  QuadStatus.swift
//  QuadController
//
//  Created by Brad Hesse on 1/23/21.
//

import Foundation

public class QuadStatus : ObservableObject {
  @Published var rssi : Double?
  @Published var connected = false
  @Published var armed = false
  @Published var motorValues : [Double]?
  @Published var batteryVoltage : Double?
  @Published var batteryPercent : Double?
  @Published var yaw : Double?
  @Published var pitch : Double?
  @Published var roll : Double?
  
  public func updateTelemetry(withData data : Data) {
    let dataString = String(data: data, encoding: .utf8)
    guard let values = dataString?.components(separatedBy: ","), values.count >= 7 else {
      print("Invalid telemetry update")
      return
    }
    
    let armedVal = values[0] == "1"
    
    var motors = [Double]()
    for i in 1..<5 {
      guard let motorVal = Double(values[i]) else {
        print("Received an invalid telemetry update (no motor value)")
        return
      }
      motors.append(motorVal)
    }
    
    guard let batVoltage = Double(values[5]) else {
      print("Received invalid JSON telemetry update (no battery voltage value)")
      return
    }
    
    guard let batPct = Double(values[6]) else {
      print("Received invalid JSON telemetry update (no battery percent value)")
      return
    }

    self.armed = armedVal
    self.motorValues = motors
    self.batteryVoltage = batVoltage
    self.batteryPercent = batPct
    
    if (values.count > 7) {
      guard let yaw = Double(values[7]), let pitch = Double(values[8]), let roll = Double(values[9]) else {
        return;
      }
      
      self.yaw = yaw;
      self.pitch = pitch;
      self.roll = roll;
    }
  }
}
