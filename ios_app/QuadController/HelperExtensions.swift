//
//  HelperExtensions.swift
//  QuadController
//
//  Created by Brad Hesse on 10/25/20.
//

import UIKit
import AudioToolbox
import CryptoKit

extension Data {
  var md5Hash : String {
    let digest = Insecure.MD5.hash(data: self)
    // Convert the digest to a hex string
    return digest.map { String(format: "%02x", $0) }.joined()
  }
}

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
