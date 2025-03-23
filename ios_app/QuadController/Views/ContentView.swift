//
//  ContentView.swift
//  QuadController
//
//  Created by Brad Hesse on 10/24/20.
//

import SwiftUI

private class PreviousControlValues {
  var x = 0.0
  var y = 0.0
  
  init(x : Double, y : Double) {
    self.x = x
    self.y = y
  }
}

struct ContentView: View, ControllerViewDelegate, BLEControllerDelegate {
  
  @ObservedObject var controller = BLEController()
  var pidCalibrationController : PIDController!
  @ObservedObject var accelGyroCalibrationController = CalibrationController(calibrationType: .AccelerometerGyro)
  @ObservedObject var magnetometerCalibrationController = CalibrationController(calibrationType: .Magnetometer)
  @ObservedObject var magnetometerMotorCalibrationController = CalibrationController(calibrationType: .MagnetometerMotorsCompensation)
  @State private var showingPopover = false
  @State private var showingPIDCalibrationPopover = false
  @State fileprivate var leftStickValues = PreviousControlValues(x : 0.0, y : 127.5)
  @State fileprivate var rightStickValues = PreviousControlValues(x : 127.5, y : 127.5)
  @State private var recordingDebugData = false
  @State private var showingDebugMenu = false
  
  @State private var motor1 = 0.0
  @State private var motor2 = 0.0
  @State private var motor3 = 0.0
  @State private var motor4 = 0.0
  
  
  var leftStick = ControllerView(StickConfiguration(identifier: 0,
                                                    returnsToDefaultForAxes: Set([.horizontal]),
                                                    vibrates: true,
                                                    axis: .vertical,
                                                    horizontalDefaultStickLocation: .center,
                                                    verticalDefaultStickLocation: .begin))
  var rightStick = ControllerView(StickConfiguration(identifier: 1,
                                                     returnsToDefaultForAxes: Set([.vertical, .horizontal]),
                                                     vibrates: true,
                                                     axis: .none))
  
  init() {
    self.init(bleDisabled: false)
  }
  
  init(bleDisabled : Bool) {
    pidCalibrationController = PIDController(controller: controller)
    leftStick.controlDelegate = self
    rightStick.controlDelegate = self
    if (!bleDisabled) {
      controller.scan()
    }
    controller.delegate = self
    accelGyroCalibrationController.provideBLEController(controller)
    magnetometerCalibrationController.provideBLEController(controller)
    magnetometerMotorCalibrationController.provideBLEController(controller)
  }
  
  func calibrationButton(type : CalibrationType, typeLabel : String, calibrationController : CalibrationController) -> some View
  {
    Button("Calibrate \(typeLabel)") {
      controller.sendCalibrationUpdate(forSensorType: type, response: .Start)
    }
    .alert(calibrationController.currentAlert?.title ?? "None", isPresented: .constant(calibrationController.currentAlert != nil)) {
      if let buttons = calibrationController.currentAlert?.buttons {
        ForEach(buttons, id: \.text) { button in
            Button(button.text) {
                button.action()
            }
        }
      } else {
        Button("Done") {
          calibrationController.currentAlert = nil
        }
      }
    }
  }
  
  var body: some View {
    ZStack {
      VStack {
        HStack {
          Spacer()
          leftStick
          Spacer()
          BLEStatusView(controller)
          Spacer()
          rightStick
          Spacer()
        }
        HStack {
          Button("PIDs"){
            showingPIDCalibrationPopover = true
          }
          .popover(isPresented: $showingPIDCalibrationPopover) {
            PIDAllAxisView(controller: pidCalibrationController, initialGains: pidCalibrationController.gains)
          }
          Spacer()
          Button("Motor Debug") {
            showingPopover = true
          }
          .popover(isPresented: $showingPopover) {
            VStack {
              HStack {
                Text("Motor 1: ")
                Slider(value: $motor1, in: 0.0...1.0) { editing in
                  if !editing {
                    motor1 = 0.0
                  }
                }.onChange(of: motor1, { oldValue, newValue in
                  updated(motor: 1, value: motor1)
                })
              }
              HStack {
                Text("Motor 2: ")
                Slider(value: $motor2, in: 0.0...1.0) { editing in
                  if !editing {
                    motor2 = 0.0
                  }
                }.onChange(of: motor2, { oldValue, newValue in
                  updated(motor: 2, value: motor2)
                })
              }
              HStack {
                Text("Motor 3: ")
                Slider(value: $motor3, in: 0.0...1.0) { editing in
                  if !editing {
                    motor3 = 0.0
                  }
                }.onChange(of: motor3, { oldValue, newValue in
                  updated(motor: 3, value: motor3)
                })
              }
              HStack {
                Text("Motor 4: ")
                Slider(value: $motor4, in: 0.0...1.0) { editing in
                  if !editing {
                    motor4 = 0.0
                  }
                }.onChange(of: motor4, { oldValue, newValue in
                  updated(motor: 4, value: motor4)
                })
              }
              Button("Done") {
                changeMotorDebugState(editing: false)
                showingPopover = false
              }
            }
          }
          .onChange(of: showingPopover, { oldValue, newValue in
            changeMotorDebugState(editing: showingPopover)
          })
          Spacer()
          calibrationButton(type: .AccelerometerGyro, typeLabel: "Accel+Gyro", calibrationController: accelGyroCalibrationController)
          Spacer()
          calibrationButton(type: .Magnetometer, typeLabel: "Magnetometer", calibrationController: magnetometerCalibrationController)
          Spacer()
          calibrationButton(type: .MagnetometerMotorsCompensation, typeLabel: "Mag-Motor", calibrationController: magnetometerMotorCalibrationController)
          Spacer()
          Button(recordingDebugData ? "Stop Recording" : "Start Recording") {
            recordingDebugData = !recordingDebugData
            controller.recordDebugStateChange(record: recordingDebugData)
            if (recordingDebugData) {
              CameraController.shared.startRecording()
            } else {
              CameraController.shared.stopRecording()
            }
          }
          Spacer()
          Button("Debug Menu") {
            showingDebugMenu = true
          }.popover(isPresented: $showingDebugMenu) {
            DebuggerView(controller: controller) {
              showingDebugMenu = false
            }
          }
          Spacer()
        }
        .frame(maxWidth: .infinity, maxHeight: 44, alignment: .center)
      }
      .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .center)
      CameraPreview()
        .frame(width: 120, height: 67.5, alignment: .topTrailing)
        .cornerRadius(12)
        .clipped()
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topTrailing)
        .padding(.top, 16)
        .padding(.trailing, 16)
    }
  }
  
  private func changeMotorDebugState(editing: Bool) {
    controller.updateMotorDebugStatus(inProgress: editing)
  }
  
  private func updated(motor: Int, value: Double) {
    controller.manualMotorDebugSpeed(motor: motor, speed: value)
  }
  
  private func scaled(_ value : Double) -> Double {
    return (value * 255.0)
  }
  
  // ControllerViewDelegate - Functions
  func updatedStickLocation(_ sender : ControllerView, x : Double, y : Double, endGesture : Bool) {
    if (sender.controllerId == leftStick.controllerId) {
      leftStickValues.x = scaled(x)
      leftStickValues.y = scaled(y)
    } else {
      rightStickValues.x = scaled(x)
      rightStickValues.y = scaled(y)
    }
    controller.sendControlUpdate(
      StickUpdateParams(
        throttle: leftStickValues.y,
        yaw: leftStickValues.x,
        pitch: rightStickValues.y,
        roll: rightStickValues.x
      ),
      endGesture: endGesture
    )
  }
  
  // TODO: do something with these
  func didDisconnect() {
    
  }
  
  func didConnect() {
    
  }
}

struct ContentView_Previews: PreviewProvider {
  static var previews: some View {
    Group {
      ContentView(bleDisabled: true)
        .previewLayout(.fixed(width: 1000, height: 500))
    }
  }
}
