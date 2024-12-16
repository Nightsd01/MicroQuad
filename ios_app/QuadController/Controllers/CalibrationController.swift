//
//  CalibrationController.swift
//  QuadController
//
//  Created by Brad Hesse on 11/30/24.
//

import Foundation

import SwiftUI

class CalibrationController : ObservableObject, BLESensorCalibrationDelegate {
  private var bleController : BLEController?
  
  public let calibrationSensorType : CalibrationType
  
  @Published public var calibrationAlert : String?
  @Published public var alertButtonsAndHandlers : [String : () -> Void]?
  
  init(calibrationType : CalibrationType) {
    calibrationSensorType = calibrationType
  }
  
  func provideBLEController(_ controller : BLEController) {
    self.bleController = controller
    self.bleController?.addCalibrationDelegate(forSensorType: calibrationSensorType, calibrationDelegate: self)
  }
  
  private func updateAlertActionHandlers(_ request : CalibrationRequest) {
    switch request {
      case .PlaceFlat,
          .PitchUp,
          .PitchDown,
          .UpsideDown,
          .RollRight,
          .RollLeft,
          .Roll360:
          alertButtonsAndHandlers = [
            "Next" : { [weak self] in
              guard let sensorType = self?.calibrationSensorType else {
                return
              }
              self?.bleController?.sendCalibrationUpdate(forSensorType: sensorType, response: .Continue)
            },
            "Cancel" : { [weak self] in
              guard let sensorType = self?.calibrationSensorType else {
                return
              }
              self?.bleController?.sendCalibrationUpdate(forSensorType: sensorType, response: .Cancel)
            }
          ]
          break
        case .Complete, .Failed:
          alertButtonsAndHandlers = [
            "Done" : {}
          ]
        default:
          fatalError("Received unacceptable CalibrationRequest case")
    }
  }
  
  // MARK: BLESensorCalibrationDelegate Functions
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
      case .Roll360:
        alert = "Roll the device across all orientations"
        break
      case .Complete:
        alert = "Calibration completed successfully"
        return
      case .Failed:
        alert = "Calibration failed"
        return
      default:
        fatalError("Received unacceptable CalibrationRequest case")
    }
    updateAlertActionHandlers(request)
    print("Showing calibration alert: \(alert ?? "Invalid")")
    calibrationAlert = alert

  }
}
