//
//  WeakReference.swift
//  QuadController
//
//  Created by Brad Hesse on 12/9/24.
//

import Foundation

class WeakReference<T: AnyObject> {
    weak var value: T?
    init(_ value: T) {
        self.value = value
    }
}
