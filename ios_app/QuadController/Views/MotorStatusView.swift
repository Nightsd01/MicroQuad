//
//  MotorRPMView.swift
//  QuadController
//
//  Created by Brad Hesse on 1/30/21.
//

import SwiftUI
import UIKit
import DGCharts

struct MotorStatusView : View {
  var status : QuadStatus
  let motorId : Int
  
  init(status : QuadStatus, motorId : Int) {
    self.status = status
    self.motorId = motorId
  }
  
  var body: some View {
    ZStack(alignment: .center) {
      MotorThrottleView(status: status, motorId: motorId)
        .cornerRadius(12.0)
        .clipped()
      Text(String(self.motorId))
        .frame(maxWidth: .infinity, alignment: .center)
        .foregroundColor(.white)
    }
    .cornerRadius(12.0)
    .clipped()
  }
}

struct MotorThrottleView : UIViewRepresentable {
  static let initTime = Date()
  
  typealias UIViewType = LineChartView
  
  @ObservedObject var status : QuadStatus
  let motorId : Int
  
  init(status : QuadStatus, motorId : Int) {
    self.status = status
    self.motorId = motorId
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
    leftAxis.axisMinimum = 900
    leftAxis.drawAxisLineEnabled = false
    dataSet.axisDependency = .left
    leftAxis.axisMaximum = 2000
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
  
  func updateUIView(_ uiView: LineChartView, context: UIViewRepresentableContext<MotorThrottleView>) {
    let dataSet = (uiView.lineData?.dataSets.first) as! LineChartDataSet
    
    let motorValue = self.status.motorValues?[self.motorId - 1] ?? 1000.0
    dataSet.append(ChartDataEntry(x: Date().timeIntervalSince(MotorThrottleView.initTime), y: motorValue))
    
    if (dataSet.count > 50) {
      dataSet.removeEntry(index: 0)
    }
    
    uiView.lineData?.notifyDataChanged()
    uiView.notifyDataSetChanged()
  }
}
