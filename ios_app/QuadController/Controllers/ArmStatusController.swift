//
//  ArmStatusController.swift
//  QuadController
//
//  Created by Brad Hesse on 5/9/25.
//

import Foundation

public protocol ArmStatusListener : AnyObject {
  func handleArmStatusChange(sendUpdateToQuadcopter : Bool)
}

public class ArmStatusController: ObservableObject, ArmStatusListener {
  @Published public var armed = false
  
  var bleController : BLEController!
  
  init(armed: Bool = false) {
    self.armed = armed
  }
  
  func provideBLEController(_ bleController: BLEController)
  {
    self.bleController = bleController
  }

  public func handleArmStatusChange(sendUpdateToQuadcopter : Bool) {
    armed = !armed
    if (sendUpdateToQuadcopter) {
      bleController.updateArmStatus(armed: armed)
    }
  }
  
}
