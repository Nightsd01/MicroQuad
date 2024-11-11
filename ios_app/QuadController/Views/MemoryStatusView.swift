//
//  MemoryStatusView.swift
//  QuadController
//
//  Created by Brad Hesse on 1/30/21.
//

import SwiftUI
import UIKit
import DGCharts

struct MemoryStatusView : View {
  @ObservedObject var status : QuadStatus
  
  init(status : QuadStatus) {
    self.status = status
  }
  
  var body: some View {
    ZStack(alignment: .center) {
      MemoryDebugView(status: status)
        .cornerRadius(12.0)
        .clipped()
      if let memoryStatus = status.memoryStatus {
        VStack {
          Spacer()
          HStack {
            Text(String(format: "%.2fMB Current\n%.2fMB Peak", Float(memoryStatus.totalHeapSizeByteCount - memoryStatus.freeHeapByteCount) / Float((1024 * 1024)), Float(memoryStatus.totalHeapSizeByteCount - memoryStatus.minFreeHeapByteCount) / Float((1024 * 1024))))
              .frame(maxWidth: .infinity, alignment: .leading)
              .foregroundColor(.white)
              .padding(EdgeInsets(top: 2, leading: 8, bottom: 2, trailing: 8))
            Text(String(format: "%.2f%%", 100.0 * Float(memoryStatus.totalHeapSizeByteCount - memoryStatus.freeHeapByteCount) / Float(memoryStatus.totalHeapSizeByteCount)))
              .bold()
              .foregroundColor(.white)
              .padding(EdgeInsets(top: 2, leading: 8, bottom: 2, trailing: 8))
          }
          Spacer()
        }
      } else {
        Text("Memory Usage (None)")
          .frame(maxWidth: .infinity, alignment: .leading)
          .foregroundColor(.white)
          .padding(8)
      }
    }
    .cornerRadius(12.0)
    .clipped()
  }
}

struct MemoryDebugView : UIViewRepresentable {
  static let initTime = Date()
  
  typealias UIViewType = LineChartView
  
  @ObservedObject var status : QuadStatus
  
  init(status : QuadStatus) {
    self.status = status
  }
  
  func makeUIView(context: Context) -> LineChartView {
    let chart = LineChartView()
    let dataSet = LineChartDataSet()
    let data = LineChartData(dataSet: dataSet)
    dataSet.lineWidth = 2.0
    chart.setViewPortOffsets(left: 0.0, top: 0.0, right: 0.0, bottom: 0.0)
    
    chart.noDataText = "No Data"
    dataSet.colors = [.red, .red, .clear]
    data.setDrawValues(true)
    
    dataSet.drawFilledEnabled = true
    
    chart.autoScaleMinMaxEnabled = true
    chart.legend.form = .none
    dataSet.drawCirclesEnabled = false
    dataSet.lineWidth = 2.0
    dataSet.fillColor = .red
    chart.scaleXEnabled = true
    chart.scaleYEnabled = true
    chart.legend.enabled = false
    
    chart.rightAxis.enabled = false
    
    let leftAxis = chart.leftAxis
    leftAxis.drawGridLinesEnabled = false
    leftAxis.granularityEnabled = true
    leftAxis.drawLabelsEnabled = false
    leftAxis.axisMinimum = 0
    leftAxis.drawAxisLineEnabled = false
    dataSet.axisDependency = .left
    leftAxis.axisMaximum = 100
    chart.xAxis.drawLabelsEnabled = false
    chart.xAxis.drawGridLinesEnabled = false
    chart.xAxis.drawAxisLineEnabled = false
    chart.borderColor = .clear
    chart.data = data
    
    dataSet.mode = .cubicBezier
    dataSet.drawValuesEnabled = false
    
    chart.setContentHuggingPriority(.defaultHigh, for: .vertical)
    chart.setContentHuggingPriority(.defaultHigh, for: .horizontal)
    chart.layoutMargins = .zero
    
    chart.backgroundColor = .darkGray
    
    return chart
  }
  
  func updateUIView(_ uiView: LineChartView, context: UIViewRepresentableContext<MemoryDebugView>) {
    // Safely unwrap the dataSet
    guard let dataSet = uiView.data?.dataSets.first as? LineChartDataSet else {
        return
    }

    // Ensure memoryStatus is available
    guard let memoryStatus = status.memoryStatus else {
        return
    }

    // Calculate memory pressure
    let memoryPressure = 100.0 * Float(memoryStatus.totalHeapSizeByteCount - memoryStatus.freeHeapByteCount) / Float(memoryStatus.totalHeapSizeByteCount)

    // Create a new ChartDataEntry
    let entry = ChartDataEntry(x: Date().timeIntervalSince(MemoryDebugView.initTime), y: Double(memoryPressure))

    // Add the entry to the dataSet
    dataSet.addEntry(entry)

    // Keep only the latest 50 entries
    if dataSet.entryCount > 50 {
      dataSet.removeEntry(dataSet.first!)
    }

    // Notify the chart data and chart view that the data has changed
    uiView.data?.notifyDataChanged()
    uiView.notifyDataSetChanged()
  }
}
