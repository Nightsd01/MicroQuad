//
//  StickUpdateParams.swift
//  QuadController
//
//  Created by Brad Hesse on 1/23/21.
//

import Foundation

public struct StickUpdateParams {
  public var throttle : Double
  public var yaw : Double
  public var pitch : Double
  public var roll : Double
  public var armed : Bool
  
  init(throttle : Double, yaw : Double, pitch : Double, roll : Double) {
    self.throttle = throttle
    self.yaw = yaw
    self.pitch = pitch
    self.roll = roll
    self.armed = false
  }
  
  func toData() throws -> Data {
    return "\(throttle),\(yaw),\(pitch),\(roll)".data(using: .utf8)!
  }
}
