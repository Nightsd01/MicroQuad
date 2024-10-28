//
//  BatteryStatusView.swift
//  QuadController
//
//  Created by Brad Hesse on 1/31/21.
//

import SwiftUI

struct BatteryStatusView : View {
  @ObservedObject var status : QuadStatus
  
  init(_ quadStatus : QuadStatus) {
    status = quadStatus
  }

  var pct : Double {
    let batPct = status.batteryPercent ?? 0.0
    if (batPct.isNaN || batPct < 0.0) {
      return 0.0
    } else {
      return batPct
    }
  }
  
  var body: some View {
    let batPercent = pct
    
    let color : Color = batPercent >= 0.8 ? .green : (batPercent > 0.4 ? .yellow : .red)
    
    GeometryReader { geometry in
      ZStack(alignment: .leading) {
        RoundedRectangle(cornerRadius: 12.0, style: .circular)
          .foregroundColor(.gray)
          .frame(maxWidth: .infinity, maxHeight: 32, alignment: .center)
        RoundedRectangle(cornerRadius: 12.0, style: .circular)
          .foregroundColor(color)
          .frame(width: geometry.size.width * CGFloat(batPercent), height: geometry.frame(in: .local).height)
          .padding(.top, 0.0)
          .padding(.leading, 0.0)
        Text("\(String(format: "%.2f", status.batteryVoltage ?? 0.0))V")
          .bold()
          .foregroundColor(.white)
          .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .center)
      }
      .frame(maxWidth: .infinity, maxHeight: 32, alignment: .center)
    }
  }
}
