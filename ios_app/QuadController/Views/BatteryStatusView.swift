//
//  BatteryStatusView.swift
//  QuadController
//
//  Created by Brad Hesse on 1/31/21.
//

import SwiftUI

struct BatteryStatusView : View {
  @ObservedObject var status : QuadStatus
  @State private var animationOffset: CGFloat = 0
  
  init(_ quadStatus : QuadStatus) {
    status = quadStatus
  }

  var pct : Double {
    let batPct = status.batteryPercent ?? 0.0
    if (batPct.isNaN || batPct < 0.0) {
      return 0.0
    } else {
      return Double(batPct)
    }
  }
  
  var batteryEmoji: String {
    guard let batteryStatus = status.batteryStatus else {
      return "\u{1F50B}"
    }
    
    switch batteryStatus {
    case .Charging:
      return "\u{1F50C}"
    case .ChargeComplete:
      return "\u{2705}"
    case .LowBattery:
      return "\u{1FAAB}"
    case .Fault:
      return "\u{26A0}\u{FE0F}"
    case .NoInputPower:
      return "\u{1F50B}"
    case .ShutdownOrNoBattery:
      return "\u{274C}"
    default:
      return "\u{1F50B}"
    }
  }
  
  var isCharging: Bool {
    guard let batteryStatus = status.batteryStatus else {
      return false
    }
    return batteryStatus == .Charging
  }
  
  var body: some View {
    let batPercent = pct
    
    let color : Color = (status.batteryStatus == .Charging
                         ? .green
                         : (batPercent >= 0.7
                            ? .green
                            : (batPercent > 0.4 ? .yellow : .red)))
    
    GeometryReader { geometry in
      ZStack(alignment: .leading) {
        // Background
        RoundedRectangle(cornerRadius: 12.0, style: .circular)
          .foregroundColor(.gray.opacity(0.3))
          .frame(maxWidth: .infinity, maxHeight: 32, alignment: .center)
        
        // Static fill bar - This will now be the main battery level indicator
        RoundedRectangle(cornerRadius: 12.0, style: .circular)
          .fill(color) // Use solid color or a consistent opacity like .opacity(0.8)
          .frame(width: geometry.size.width * CGFloat(batPercent), height: geometry.frame(in: .local).height)
        
        // Content overlay
        HStack(spacing: 4) {
          Text(batteryEmoji)
            .font(.system(size: 16))
          Text("\(String(format: "%.2f", status.batteryVoltage ?? 0.0))V")
            .bold()
            .foregroundColor(.white)
            .shadow(color: .black.opacity(0.3), radius: 1, x: 1, y: 1)
        }
        .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .center)
      }
      .frame(maxWidth: .infinity, maxHeight: 32, alignment: .center)
    }
  }
}
