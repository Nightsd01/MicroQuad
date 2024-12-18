//
//  AlertModels.swift
//  QuadController
//
//  Created by Brad Hesse on 12/15/24.
//

import SwiftUI

struct AlertData {
  let title : String
  let buttons : [AlertButton]
}

struct AlertButton {
  let text : String
  let action : () -> Void
}
