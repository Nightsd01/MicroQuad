//
//  CameraPreview.swift
//  QuadController
//
//  Created by Brad Hesse on 11/29/24.
//

import Foundation
import UIKit
import SwiftUI
import AVFoundation

struct CameraPreview: UIViewRepresentable {
  func makeCoordinator() -> Coordinator {
    Coordinator(self)
  }
  
  func makeUIView(context: Context) -> UIView {
    let view = context.coordinator.view
    
    CameraController.shared.addCameraPreviewUpdateHandler { previewLayer in
      previewLayer.videoGravity = .resizeAspectFill
      
      // Initial orientation
      previewLayer.connection?.videoOrientation = videoOrientation()
      
      view.layer.addSublayer(previewLayer)
      previewLayer.frame = view.bounds
    }
    
    // Start observing orientation changes
    NotificationCenter.default.addObserver(
      context.coordinator,
      selector: #selector(Coordinator.orientationChanged),
      name: UIDevice.orientationDidChangeNotification,
      object: nil
    )
    
    return view
  }
  
  func updateUIView(_ uiView: UIView, context: Context) {
    CameraController.shared.addCameraPreviewUpdateHandler { previewLayer in
      previewLayer.frame = uiView.bounds
    }
  }
  
  private func videoOrientation() -> AVCaptureVideoOrientation {
    let orientation = UIDevice.current.orientation
    switch orientation {
    case .landscapeLeft: return .landscapeRight
    case .landscapeRight: return .landscapeLeft
    case .portraitUpsideDown: return .portraitUpsideDown
    default: return .portrait
    }
  }
  
  class Coordinator {
    let view: UIView
    
    init(_ parent: CameraPreview) {
      view = PreviewView()
    }
    
    @objc func orientationChanged() {
      CameraController.shared.addCameraPreviewUpdateHandler { previewLayer in
        // Need to get the current orientation
        let orientation = UIDevice.current.orientation
        
        switch orientation {
        case .landscapeLeft:
          previewLayer.connection?.videoOrientation = .landscapeRight
        case .landscapeRight:
          previewLayer.connection?.videoOrientation = .landscapeLeft
        default:
          previewLayer.connection?.videoOrientation = .landscapeRight
        }
      }
    }
    
    deinit {
      NotificationCenter.default.removeObserver(self)
    }
  }
  
  class PreviewView: UIView {
    override var contentMode: UIView.ContentMode {
      get { .scaleAspectFill }
      set { }
    }
    
    override func layoutSubviews() {
      super.layoutSubviews()
      layer.sublayers?.first?.frame = bounds
    }
  }
}
