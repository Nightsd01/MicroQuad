//
//  CalibrationController.swift
//  QuadController
//
//  Created by Brad Hesse on 11/30/24.
//

import Foundation

import SwiftUI

class CalibrationController : ObservableObject, BLEAccelerometerCalibrationDelegate {
  private var bleController : BLEController?
  
  @Published public var calibrationAlert : String?
  @Published public var alertButtonsAndHandlers : [String : () -> Void]?
  
  func provideBLEController(_ controller : BLEController) {
    self.bleController = controller
    self.bleController?.calibrationDelegate = self
  }
  
  // MARK: BLEAccelerometerCalibrationDelegate Functions
  func didGetCalibrationRequest(_ request: CalibrationRequest) {
    var alert : String!
    switch request {
      case .PlaceFlat:
        alert = "Place flat"
        break
      case .PitchUp:
        alert = "Pitch up 90 degrees"
        break
      case .PitchDown:
        alert = "Pitch down 90 degrees"
        break
      case .UpsideDown:
        alert = "Upside down"
        break
      case .RollRight:
        alert = "Roll right 90 degrees"
        break
      case .RollLeft:
        alert = "Roll left 90 degrees"
        break
      case .Complete:
        calibrationAlert = "Calibration completed successfully"
        return
      case .Failed:
        calibrationAlert = "Calibration failed"
        return
      default:
        fatalError("Received unacceptable CalibrationRequest case")
    }
    
    print("Showing calibration alert: \(alert ?? "Invalid")")
    calibrationAlert = alert
    alertButtonsAndHandlers = [
      "Next" : { [weak self] in
        self?.bleController?.continueAccelerometerCalibration(.Continue)
      },
      "Cancel" : { [weak self] in
        self?.bleController?.continueAccelerometerCalibration(.Cancel)
      }
    ]
  }
}
