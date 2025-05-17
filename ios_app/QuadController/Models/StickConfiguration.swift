//
//  StickConfiguration.swift
//  QuadController
//
//  Created by Brad Hesse on 10/25/20.
//

import Foundation

public enum PrimaryStickLocation {
  case begin
  case center
  case end
}

public enum StickType {
  case left
  case right
}

public class StickConfiguration : ObservableObject {
  let identifier : Int
  let stickAxis : Axis
  let returnsToDefaultForAxes : Set<Axis>
  let vibrates : Bool
  let tolerance : Double
  let horizontalDefaultStickLocation : PrimaryStickLocation
  let verticalDefaultStickLocation : PrimaryStickLocation
  let stickType : StickType
  
  init(identifier : Int,
       returnsToDefaultForAxes : Set<Axis> = Set<Axis>(),
       vibrates : Bool,
       axis : Axis = .none,
       tolerance : Double = 20.0,
       stickType: StickType,
       horizontalDefaultStickLocation : PrimaryStickLocation = .center,
       verticalDefaultStickLocation : PrimaryStickLocation = .center) {
    self.identifier = identifier
    self.returnsToDefaultForAxes = returnsToDefaultForAxes
    self.stickAxis = axis
    self.vibrates = vibrates
    self.tolerance = tolerance
    self.stickType = stickType
    self.horizontalDefaultStickLocation = horizontalDefaultStickLocation
    self.verticalDefaultStickLocation = verticalDefaultStickLocation
  }
}
