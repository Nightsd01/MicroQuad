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
  @State private var showingPopover = false
  @State fileprivate var leftStickValues = PreviousControlValues(x : 0.0, y : 127.5)
  @State fileprivate var rightStickValues = PreviousControlValues(x : 127.5, y : 127.5)
  @State private var recordingDebugData = false
  @State private var showingDebugMenu = false
  @State private var calibrating = false

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
    leftStick.controlDelegate = self
    rightStick.controlDelegate = self
    if (!bleDisabled) {
      controller.scan()
    }
    controller.delegate = self
  }
  
  var body: some View {
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
        Spacer()
        Button("Motor Debug Menu") {
          showingPopover = true
        }.popover(isPresented: $showingPopover) {
          VStack {
            HStack {
              Text("Motor 1: ")
              Slider(value: $motor1, in: 0.0...1.0) { editing in
                if !editing {
                  motor1 = 0.0
                }
              }.onChange(of: motor1, perform: { value in
                updated(motor: 1, value: motor1)
              })
            }
            HStack {
              Text("Motor 2: ")
              Slider(value: $motor2, in: 0.0...1.0) { editing in
                if !editing {
                  motor2 = 0.0
                }
              }.onChange(of: motor2, perform: { value in
                updated(motor: 2, value: motor2)
              })
            }
            HStack {
              Text("Motor 3: ")
              Slider(value: $motor3, in: 0.0...1.0) { editing in
                if !editing {
                  motor3 = 0.0
                }
              }.onChange(of: motor3, perform: { value in
                updated(motor: 3, value: motor3)
              })
            }
            HStack {
              Text("Motor 4: ")
              Slider(value: $motor4, in: 0.0...1.0) { editing in
                if !editing {
                  motor4 = 0.0
                }
              }.onChange(of: motor4, perform: { value in
                updated(motor: 4, value: motor4)
              })
            }
            Button("Done") {
              changeMotorDebugState(editing: false)
              showingPopover = false
            }
          }
        }
        .onChange(of: showingPopover, perform: { value in
          changeMotorDebugState(editing: showingPopover)
        })
        Spacer()
        Button("Calibrate") {
          controller.calibrate()
        }
        Spacer()
        Button(recordingDebugData ? "Stop Recording" : "Start Recording") {
          recordingDebugData = !recordingDebugData
          controller.recordDebugStateChange(record: recordingDebugData)
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
    .alert("Calibration Complete", isPresented: .constant(controller.connected && controller.calibrated && controller.calibrationValues != nil)) {
      Button("OK") {
        controller.calibrated = false
        controller.calibrationValues = nil
      }
        } message: {
          if (controller.calibrationValues == nil) {
            Text("")
          } else {
            Text(
              "Gyro Bias: \(controller.calibrationValues!.gyroBiasX), \(controller.calibrationValues!.gyroBiasY), \(controller.calibrationValues!.gyroBiasZ)"
              + "\nAccel Bias: \(controller.calibrationValues!.accelBiasX), \(controller.calibrationValues!.accelBiasY), \(controller.calibrationValues!.accelBiasZ)"
              + "\nAccel Scale Factor: \(controller.calibrationValues!.accelScaleX), \(controller.calibrationValues!.accelScaleY), \(controller.calibrationValues!.accelScaleZ)"
              + "\nGyro Offsets: \(controller.calibrationValues!.gyroOffsetX), \(controller.calibrationValues!.gyroOffsetY), \(controller.calibrationValues!.gyroOffsetZ)"
              + "\nnAccel Offsets: \(controller.calibrationValues!.accelOffsetX), \(controller.calibrationValues!.accelOffsetY), \(controller.calibrationValues!.accelOffsetZ)"
            )
          }
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
