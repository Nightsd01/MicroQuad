//
//  PIDDebuggerView.swift
//  QuadController
//
//  Created by Brad Hesse on 3/18/25.
//

import SwiftUI

func axisName(axis : ControlAxis) -> String
{
  switch (axis) {
    case .Yaw:
      return "Yaw"
    case .Pitch:
      return "Pitch"
    case .Roll:
      return "Roll"
    default:
      fatalError("Undefined control axis \(axis)")
  }
}

func pidTypeName(pidType: PIDType) -> String
{
  switch (pidType) {
    case .Angle:
      return "Angle"
    case .Rate:
      return "Rate"
  }
}

struct PIDSlidersView: View {
  // Title shown at the top of the view (e.g. "Angle PID" or "Rate PID")
  let axis : ControlAxis
  let pidType : PIDType
  let changeHandler : (PIDType, ControlAxis, PIDValues) -> Void
  
  // Bindings to the actual values for P, I, and D
  @State var pValue: Double
  @State var iValue: Double
  @State var dValue: Double
  
  init(axis: ControlAxis, pidType: PIDType, changeHandler: @escaping (PIDType, ControlAxis, PIDValues) -> Void, initialValues: PIDValues) {
    self.axis = axis
    self.pidType = pidType
    self.changeHandler = changeHandler
    self.pValue = Double(initialValues.proportional)
    self.iValue = Double(initialValues.integral)
    self.dValue = Double(initialValues.derivative)
  }
  
  // Each slider can have its own adjustable value range
  let pRange : ClosedRange<Double> = 0.0...10.0
  let iRange : ClosedRange<Double> = 0.0...1.0
  let dRange : ClosedRange<Double> = 0.0...0.1
  
  func getValues() -> PIDValues {
    PIDValues(proportional: Float32(pValue), integral: Float32(iValue), derivative: Float32(dValue))
  }
  
  var body: some View {
    VStack(alignment: .leading, spacing: 16) {
      // Title at the top
      Text(axisName(axis: axis))
        .font(.headline)
      
      // Sliders for P, I, D
      VStack(alignment: .leading, spacing: 12) {
        
        // P
        Text("P: \(pValue, specifier: "%.3f")")
        Slider(value: $pValue, in: pRange)
          .onChange(of: pValue) { oldValue, newValue in
            changeHandler(pidType, axis, getValues())
          }
        
        // I
        Text("I: \(iValue, specifier: "%.3f")")
        Slider(value: $iValue, in: iRange)
          .onChange(of: iValue) { oldValue, newValue in
            changeHandler(pidType, axis, getValues())
          }
        
        // D
        Text("D: \(dValue, specifier: "%.3f")")
        Slider(value: $dValue, in: dRange)
          .onChange(of: dValue) { oldValue, newValue in
            changeHandler(pidType, axis, getValues())
          }
      }
    }
    .padding()
  }
}

struct PIDDebuggerView: View {
  let pidType : PIDType
  let changeHandler : (PIDType, ControlAxis, PIDValues) -> Void
  let gains : PIDGains
  
  init(pidType: PIDType, changeHandler: @escaping (PIDType, ControlAxis, PIDValues) -> Void, initialGains: PIDGains) {
    self.pidType = pidType
    self.changeHandler = changeHandler
    self.gains = initialGains
  }
  
  var body: some View {
    VStack {
      Text(pidTypeName(pidType: pidType))
        .font(.title3)
      PIDSlidersView(
        axis: .Yaw,
        pidType: pidType,
        changeHandler: changeHandler,
        initialValues: gains.yawGains
      )
      PIDSlidersView(
        axis: .Pitch,
        pidType: pidType,
        changeHandler: changeHandler,
        initialValues: gains.pitchGains
      )
      PIDSlidersView(
        axis: .Roll,
        pidType: pidType,
        changeHandler: changeHandler,
        initialValues: gains.rollGains
      )
    }
  }
}

struct PIDAllAxisView: View {
  let controller : PIDController
  let initialGains : PIDsContainer
  @State var pids : PIDsContainer
  
  init(controller: PIDController, initialGains: PIDsContainer) {
    self.controller = controller
    self.initialGains = initialGains
    self.pids = initialGains
  }
  
  func updateGains(existing : inout PIDGains, newValues : PIDValues, type : PIDType, axis : ControlAxis)
  {
    switch (axis) {
    case .Yaw:
      existing.yawGains = newValues
      break
    case .Pitch:
      existing.pitchGains = newValues
      break
    case .Roll:
      existing.rollGains = newValues
      break
    default:
      fatalError("Unknown axis: \(axis)")
    }
  }
  
  var body: some View {
    let changeHandler : (PIDType, ControlAxis, PIDValues) -> Void = { type, axis, values in
      switch (type) {
        case .Angle:
          self.updateGains(existing: &self.pids.angleValues, newValues: values, type: .Angle, axis: axis)
          controller.updatePids(axis: axis, type: type, newGains: self.pids.angleValues)
          break
        case .Rate:
          self.updateGains(existing: &self.pids.rateValues, newValues: values, type: .Rate, axis: axis)
          controller.updatePids(axis: axis, type: type, newGains: self.pids.rateValues)
          break
      default:
        fatalError("Unknown PID type \(type)")
      }
    }
    ScrollView {
      VStack {
        HStack {
          PIDDebuggerView(pidType: .Angle, changeHandler: changeHandler, initialGains: initialGains.angleValues)
          PIDDebuggerView(pidType: .Rate, changeHandler: changeHandler, initialGains: initialGains.rateValues)
        }
        Button("Reset to defaults") {
          pids = controller.resetToDefaults()
        }
      }
    }
  }
}
