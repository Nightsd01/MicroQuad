//
//  BLEStatusView.swift
//  QuadController
//
//  Created by Brad Hesse on 1/23/21.
//

import SwiftUI

struct BLEStatusView: View {
  var controller : BLEController
  @ObservedObject var status : QuadStatus
  @ObservedObject var armStatusController : ArmStatusController
  
  init(_ controller : BLEController, armStatusController : ArmStatusController) {
    self.controller = controller
    self.status = controller.quadStatus
    self.armStatusController = armStatusController
  }
  
  var body: some View {
    VStack {
      VStack {
        Spacer()
        HStack {
          BatteryStatusView(status)
            .frame(maxWidth: .infinity, maxHeight: 32, alignment: .center)
          Spacer()
          Button("Reset") {
            controller.resetQuad()
          }
        }
        Spacer()
        VStack(spacing: 14.0) {
          HStack(spacing: 14.0) {
            MotorStatusView(status: status, motorId: 3)
              .foregroundColor(.clear)
            MotorStatusView(status: status, motorId: 1)
              .foregroundColor(.clear)
          }
          HStack(spacing: 14.0) {
            MotorStatusView(status: status, motorId: 2)
              .foregroundColor(.clear)
            MotorStatusView(status: status, motorId: 4)
              .foregroundColor(.clear)
          }
        }
        Picker(selection: $armStatusController.armed, label: Text("Arm Status"), content: {
          Text("Disarmed").tag(false)
          Text("Armed").tag(true)
        }).pickerStyle(SegmentedPickerStyle())
        .onChange(of: armStatusController.armed) { oldValue, newValue in
          handleArmChange()
        }
        MemoryStatusView(status: status)
          .frame(height: 64)
      }
      Spacer()
      VStack{
        if let euler = status.euler {
          HStack {
            Text("IMU: ").bold()
            Text(String(format: "%.2f°  %.2f°  %.2f°  %.2fm ⇅%.2fm/s \(status.imuUpdateRateHz ?? 0)Hz",
                        euler.yaw,
                        euler.pitch,
                        euler.roll,
                        status.altitudeEstimate ?? 0.0,
                        status.verticalVelocityEstimate ?? 0.0))
              .lineLimit(1)
              .minimumScaleFactor(0.5)
          }
        } else {
          Text("No IMU").bold()
        }
        if let mag = status.magRaw {
          HStack {
            Text("Mag: ").bold()
            Text(String(format: "(%.2f°) %.2f  %.2f  %.2f, %.2f Gauss", mag.heading, mag.xyz.x, mag.xyz.y, mag.xyz.z, sqrt(pow(mag.xyz.x, 2) + pow(mag.xyz.y, 2) + pow(mag.xyz.z, 2))))
              .lineLimit(1)
              .minimumScaleFactor(0.5)
          }
        } else {
          Text("No Mag").bold()
        }
      }
      Spacer()
    }
  }
  
  private func handleArmChange() {
    armStatusController.handleArmStatusChange(sendUpdateToQuadcopter: true)
    status.armed = armStatusController.armed
  }
}


struct BLEStatusView_Previews: PreviewProvider {
  static var previews: some View {
    let controller = BLEController()
    let armStatusController = ArmStatusController()
    controller.bleStatus = "Test Status"
    controller.quadStatus.updateTelemetry(withData: "1,1150.98,1600.4,1999.0,1000.0,3.9,70".data(using: .utf8)!)
    return BLEStatusView(controller, armStatusController: armStatusController).previewLayout(.fixed(width: 1000, height: 500))
  }
}
