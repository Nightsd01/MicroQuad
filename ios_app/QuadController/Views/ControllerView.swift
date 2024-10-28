//
//  ControllerView.swift
//  QuadController
//
//  Created by Brad Hesse on 10/25/20.
//

import AudioToolbox
import SwiftUI

public protocol ControllerViewDelegate {
  func updatedStickLocation(_ sender : ControllerView, x : Double, y : Double, endGesture : Bool);
}

public struct ControllerView : View {
  private static var currentId = 0
  
  private let kSize = CGSize(width: 200, height: 200)
  
  public var controllerId : Int
  
  public var controlDelegate : ControllerViewDelegate? {
    didSet {
      print("Set control delegate!")
    }
  }
  
  @State private var dragOffset : CGPoint?
  
  @State private var withinMagneticTolerance = true
  
  private let configuration : StickConfiguration
  
  private var lastDragDate : Date?
  
  private let impactGenerator = UIImpactFeedbackGenerator(style: .medium)
  
  init(_ configuration : StickConfiguration) {
    self.configuration = configuration
    self.controllerId = ControllerView.currentId
    ControllerView.currentId += 1
  }
  
  public var body : some View {
    let controlColor = Color(red: 0.5, green: 0.5, blue: 0.5, opacity: 1.0);
    
    GeometryReader(content: { geometry in
      ZStack {
        RoundedRectangle(cornerRadius: 5.0, style: .continuous)
          .fill(controlColor)
          .frame(width: 20, height: kSize.height, alignment: .center)
        RoundedRectangle(cornerRadius: 5.0, style: .continuous)
          .fill(controlColor)
          .frame(width: kSize.width, height: 20, alignment: .center)
        Circle()
          .frame(width: /*@START_MENU_TOKEN@*/100/*@END_MENU_TOKEN@*/, height: /*@START_MENU_TOKEN@*/100/*@END_MENU_TOKEN@*/, alignment: /*@START_MENU_TOKEN@*/.center/*@END_MENU_TOKEN@*/)
          .position(offsetToCoordinate(geometry))
          .animation(.linear)
          .gesture(
            DragGesture(minimumDistance: 0.0)
              .onChanged { handleDrag($0, geometry) }
              .onEnded { handleDragEnd($0, geometry) }
          )
      }
      .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .center)
    })
  }
  
  private func offsetToCoordinate(_ geometry : GeometryProxy) -> CGPoint {
    let offset = dragOffset ?? defaultDragOffset()
    return CGPoint(x: geometry.frame(in: .local).minX + (geometry.size.width / 2.0) + offset.x,
                   y: geometry.frame(in: .local).minY + (geometry.size.height / 2.0) + offset.y)
  }
  
  private func defaultDragOffset() -> CGPoint {
    var horizontal = 0.0
    var vertical = 0.0

    switch configuration.verticalDefaultStickLocation {
      case .begin:
        vertical = 100.0
        break
      case .center:
        vertical = 0.0
        break
      case .end:
        vertical = -100.0
        break
    }
    switch configuration.horizontalDefaultStickLocation {
    case .begin:
      horizontal = 100.0
      break
    case .center:
      horizontal = 0.0
      break
    case .end:
      horizontal = -100.0
      break
    }
    return CGPoint(x: horizontal, y: vertical)
  }
  
  private func handleDragEnd(_ gesture : DragGesture.Value, _ geometry : GeometryProxy) {
    if (configuration.returnsToDefaultForAxes.count > 0) {
      let defaultOffset = defaultDragOffset()
      var newOffset = dragOffset ?? defaultOffset
      
      if (!geometry.frame(in: .global).contains(gesture.location)) {
        newOffset = defaultOffset
        
      } else {
        if (configuration.returnsToDefaultForAxes.contains(.horizontal)) {
          newOffset.x = defaultOffset.x
        }
        if (configuration.returnsToDefaultForAxes.contains(.vertical)) {
          newOffset.y = defaultOffset.y
        }
      }
      dragOffset = newOffset
      handlePositionUpdate(geometry, endGesture: true)
    }
  }
  
  private func handleDrag(_ gesture : DragGesture.Value, _ geometry : GeometryProxy) {
    let frame = geometry.frame(in: .local)
    let loc = gesture.location
    let center = CGPoint(x: frame.width / 2, y: frame.height / 2)
    let centerOffset = CGPoint(x: loc.x - center.x, y: loc.y - center.y)
    
    var newOffset = CGPoint.zero
    
    if (configuration.stickAxis == .vertical
        && Double(abs(centerOffset.x)) <= configuration.tolerance) {
      newOffset = CGPoint(x: 0.0, y: centerOffset.y)
    } else if (configuration.stickAxis == .horizontal
               && Double(abs(centerOffset.y)) <= configuration.tolerance) {
      newOffset = centerOffset
    } else if (loc.x >= frame.minX
               && loc.x <= frame.maxX
               && loc.y >= frame.minY
               && loc.y <= frame.maxY) {
      newOffset = centerOffset
    }
    
    let offset = Double(abs(configuration.stickAxis == .vertical
                              ? centerOffset.x
                              : centerOffset.y))
    
    if (configuration.vibrates && offset <= configuration.tolerance && !withinMagneticTolerance) {
      withinMagneticTolerance = true
      vibrate()
    } else if (configuration.vibrates && offset >= configuration.tolerance && withinMagneticTolerance) {
      withinMagneticTolerance = false
      vibrate()
    }
    
    newOffset.x = max(0 - (kSize.width / 2), newOffset.x)
    newOffset.x = min(kSize.width / 2, newOffset.x)
    newOffset.y = max(0 - (kSize.height / 2), newOffset.y)
    newOffset.y = min(kSize.height / 2, newOffset.y)
    dragOffset = newOffset
    
    handlePositionUpdate(geometry, endGesture: false)
  }
  
  private func handlePositionUpdate(_ geometry : GeometryProxy, endGesture : Bool) {
    guard let dragOffset = dragOffset else { return }
    let offset = CGPoint(x: (dragOffset.x + (kSize.width / 2)) / kSize.width,
                         y: 1.0 - ((dragOffset.y + (kSize.height / 2)) / kSize.height))
    controlDelegate?.updatedStickLocation(self, x: Double(offset.x), y: Double(offset.y), endGesture: endGesture)
  }
  
  private func vibrate() {
    impactGenerator.impactOccurred(intensity: 1.0)
  }
}

struct ControllerView_Previews: PreviewProvider {
  static var previews: some View {
    ControllerView(StickConfiguration(identifier: 0,
                                      returnsToDefaultForAxes: Set<Axis>([.vertical, .horizontal]),
                                      vibrates: true,
                                      axis: .none))
  }
}
