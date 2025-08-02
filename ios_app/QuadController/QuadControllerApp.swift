//
//  QuadControllerApp.swift
//  QuadController
//
//  Created by Brad Hesse on 10/24/20.
//

import SwiftUI

@main
struct QuadControllerApp: App {
  init() {
    CameraController.shared.requestCameraPermissionIfNeeded()
    GPSController.shared.requestLocationPermission()
  }
  var body: some Scene {
    WindowGroup {
      ContentView()
    }
  }
}
