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
