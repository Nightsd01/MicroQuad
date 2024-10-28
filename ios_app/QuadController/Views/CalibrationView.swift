//
//  CalibrationView.swift
//  QuadController
//
//  Created by Brad Hesse on 8/26/23.
//

import SwiftUI
import MobileCoreServices

struct CalibrationView : View {
  @ObservedObject var controller = BLEController()
  let closeBlock : () -> Void
  @State private var x: String = ""
  @State private var y: String = ""
  @State private var z: String = ""

  init(controller : BLEController, _ closeBlock : @escaping () -> Void) {
    self.controller = controller
    self.closeBlock = closeBlock
  }

  var body: some View {
    Spacer()
    VStack {
      HStack {
        Spacer()
        HStack {
          Spacer()
          Spacer()
          TextField("X", text: $x)
            .background(.gray)
            .cornerRadius(10.0)
            .padding()
            .multilineTextAlignment(.center)
            .font(Font.system(size: 20.0))
            .keyboardType(.numbersAndPunctuation)
            .onSubmit {
              controller.calibrate(axis: .x, value: Int(x) ?? 0)
            }
          Spacer()
          TextField("Y", text: $y)
            .background(.gray)
            .cornerRadius(10.0)
            .padding()
            .multilineTextAlignment(.center)
            .font(Font.system(size: 20.0))
            .keyboardType(.numbersAndPunctuation)
            .onSubmit {
              controller.calibrate(axis: .y, value: Int(y) ?? 0)
            }
          Spacer()
          TextField("Z", text: $z)
            .background(.gray)
            .cornerRadius(10.0)
            .padding()
            .multilineTextAlignment(.center)
            .font(Font.system(size: 20.0))
            .keyboardType(.numbersAndPunctuation)
            .onSubmit {
              controller.calibrate(axis: .z, value: Int(z) ?? 0)
            }
          Spacer()
          Spacer()
        }
        Spacer()
      }
      Spacer()
      Button("Close") {
        closeBlock()
      }
    }
    .padding(EdgeInsets(top: 40, leading: 40, bottom: 40, trailing: 40))
    Spacer()
  }
}
