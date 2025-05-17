//
//  CommonDefs.swift
//  QuadController
//
//  Created by Brad Hesse on 5/8/25.
//

public enum Axis {
  case vertical
  case horizontal
  case none
}

public struct XY : Comparable {
  public static func < (lhs: XY, rhs: XY) -> Bool {
    return lhs.x == rhs.x && lhs.y < rhs.y
  }
  
  let x : Float32
  let y : Float32
}

public struct XYZ {
  let x : Float32
  let y : Float32
  let z : Float32
}

public struct YawPitchRoll {
  let yaw : Float32
  let pitch : Float32
  let roll : Float32
}

public struct MagRawValues {
  let xyz: XYZ
  let heading : Float32
}

