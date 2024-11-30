//
//  HelperExtensions.swift
//  QuadController
//
//  Created by Brad Hesse on 10/25/20.
//

import UIKit
import AudioToolbox

extension UIDevice {
  static func vibrate() {
    AudioServicesPlaySystemSound(kSystemSoundID_Vibrate)
  }
}

extension UIViewController {
  func topMostViewController() -> UIViewController {
    if let presented = presentedViewController {
      return presented.topMostViewController()
    }
    if let navigation = self as? UINavigationController {
      return navigation.visibleViewController?.topMostViewController() ?? navigation
    }
    if let tab = self as? UITabBarController {
      return tab.selectedViewController?.topMostViewController() ?? tab
    }
    return self
  }
}
