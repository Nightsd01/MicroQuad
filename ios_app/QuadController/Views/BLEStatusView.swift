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
  @State private var armedStatus = 0
  
  init(_ controller : BLEController) {
    self.controller = controller
    self.status = controller.quadStatus
  }
  
  var body: some View {
    VStack {
      VStack {
        Spacer()
        HStack {
          Spacer()
          Text("Status: ").bold()
          Text(self.controller.bleStatus)
          Spacer()
        }
        Spacer()
        (status.armed
          ? Text("Armed").foregroundColor(.red).bold()
          : Text("Disarmed").foregroundColor(.green).bold()
        )
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
        Picker(selection: $armedStatus, label: Text("Arm Status"), content: {
          Text("Disarmed").tag(0)
          Text("Armed").tag(1)
        }).pickerStyle(SegmentedPickerStyle())
        .onTapGesture {
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
            Text(String(format: "%.2f°  %.2f°  %.2f°  \(status.imuUpdateRateHz ?? 0)Hz", euler.yaw, euler.pitch, euler.roll))
          }
        } else {
          Text("No IMU").bold()
        }
      }
      Spacer()
    }
  }
  
  private func handleArmChange() {
    self.armedStatus = self.armedStatus == 0 ? 1 : 0
    controller.updateArmStatus(armed: armedStatus == 1)
  }
}


struct BLEStatusView_Previews: PreviewProvider {
  static var previews: some View {
    let controller = BLEController();
    controller.bleStatus = "Test Status"
    controller.quadStatus.updateTelemetry(withData: "1,1150.98,1600.4,1999.0,1000.0,3.9,70".data(using: .utf8)!)
    return BLEStatusView(controller).previewLayout(.fixed(width: 1000, height: 500))
  }
}
