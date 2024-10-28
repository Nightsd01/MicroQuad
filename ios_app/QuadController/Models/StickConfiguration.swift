//
//  StickConfiguration.swift
//  QuadController
//
//  Created by Brad Hesse on 10/25/20.
//

import Foundation

public enum Axis {
  case vertical
  case horizontal
  case none
}

public enum PrimaryStickLocation {
  case begin
  case center
  case end
}

public class StickConfiguration : ObservableObject {
  let identifier : Int
  let stickAxis : Axis
  let returnsToDefaultForAxes : Set<Axis>
  let vibrates : Bool
  let tolerance : Double
  let horizontalDefaultStickLocation : PrimaryStickLocation
  let verticalDefaultStickLocation : PrimaryStickLocation
  
  init(identifier : Int,
       returnsToDefaultForAxes : Set<Axis> = Set<Axis>(),
       vibrates : Bool,
       axis : Axis = .none,
       tolerance : Double = 20.0,
       horizontalDefaultStickLocation : PrimaryStickLocation = .center,
       verticalDefaultStickLocation : PrimaryStickLocation = .center) {
    self.identifier = identifier
    self.returnsToDefaultForAxes = returnsToDefaultForAxes
    self.stickAxis = axis
    self.vibrates = vibrates
    self.tolerance = tolerance
    self.horizontalDefaultStickLocation = horizontalDefaultStickLocation
    self.verticalDefaultStickLocation = verticalDefaultStickLocation
  }
}
