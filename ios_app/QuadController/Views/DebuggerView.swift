//
//  DebuggerView.swift
//  QuadController
//
//  Created by Brad Hesse on 9/5/21.
//

import SwiftUI
import MobileCoreServices

struct DebuggerView : View {
  @ObservedObject var controller = BLEController()
  let closeBlock : () -> Void

  init(controller : BLEController, _ closeBlock : @escaping () -> Void) {
    self.controller = controller
    self.closeBlock = closeBlock
  }

  var body: some View {
    Spacer()
    VStack {
      if (controller.receivingDebugData && controller.debugData != nil) {
        HStack {
          Spacer()
          Spacer()
          ProgressView("Transfer Progress", value: controller.transferProgress)
          Spacer()
          Text("\(controller.debugData!.count) / \(controller.debugDataExpectedBytes)")
          Spacer()
          Spacer()
        }
      }
      HStack {
        Spacer()
        Spacer()
        Text("Debug Content")
        Spacer()
        Button("Close") {
          closeBlock()
        }
        Spacer()
        Spacer()
      }
      Spacer()
      ScrollView {
        VStack {
          Text(controller.debugDataString ?? "None")
            .fixedSize(horizontal: false, vertical: true)
            .font(.system(size: 12.0, weight: .regular, design: .monospaced))
        }
      }
      Spacer()
      HStack {
        Button("Request Debug Info") {
          controller.getDebugInfo()
        }
        Spacer()
        Button("Copy CSV") {
          if let dat = controller.debugDataString {
            UIPasteboard.general.string = dat
          }
        }
        .disabled(controller.debugDataString == nil)
        Spacer()
        Button("Copy Video") {
          CameraController.shared.shareViaAirdrop()
        }.disabled(!CameraController.shared.finishedRecording)
      }
    }
  }
}
