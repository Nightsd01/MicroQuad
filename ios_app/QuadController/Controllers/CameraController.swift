import AVFoundation
import UIKit
import SwiftUI

class CameraController: NSObject {
  static let shared = CameraController()
  private var captureSession: AVCaptureSession?
  private var videoOutput: AVCaptureMovieFileOutput?
  private var videoDevice: AVCaptureDevice?
  private var recordedVideoURL: URL?
  private var previewLayer: AVCaptureVideoPreviewLayer?
  private var cameraPreviewLayerUpdateHandlers : [(AVCaptureVideoPreviewLayer) -> Void]!
  
  
  @Published public var finishedRecording = false
  
  override init() {
    cameraPreviewLayerUpdateHandlers = []
  }
  
  func addCameraPreviewUpdateHandler(_ handler : @escaping (AVCaptureVideoPreviewLayer) -> Void)
  {
    guard let layer = previewLayer else {
      cameraPreviewLayerUpdateHandlers.append(handler)
      return
    }
    
    handler(layer)
  }
  
  // Step A: Request camera permission
  func requestCameraPermissionIfNeeded() {
    switch AVCaptureDevice.authorizationStatus(for: .video) {
    case .authorized:
      // Permission already granted
      setupCameraSession()
    case .notDetermined:
      // Request permission
      AVCaptureDevice.requestAccess(for: .video) { granted in
        if granted {
          DispatchQueue.main.async {
            self.setupCameraSession()
          }
        } else {
          print("Camera permission denied.")
        }
      }
    case .denied, .restricted:
      print("Camera access denied or restricted.")
    @unknown default:
      fatalError("Unknown authorization status")
    }
  }
  
  // Step B: Setup camera session for 240fps and 720p
  private func setupCameraSession() {
    DispatchQueue.global().async  {
      self.captureSession = AVCaptureSession()
      guard let captureSession = self.captureSession else { return }
      
      captureSession.beginConfiguration()
      captureSession.sessionPreset = .inputPriority // Set the resolution to 720p
      
      // Setup the camera device
      guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
        print("Failed to get the camera device.")
        return
      }
      self.videoDevice = device
      
      // Set 240fps
      do {
        try device.lockForConfiguration()
        print("Available 720p formats:")
        device.formats
          .filter({ format in
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            return dims.width == 1280 && dims.height == 720
          })
          .forEach({ format in
            print("Format: \(format)")
            print("Frame rates: \(format.videoSupportedFrameRateRanges.map { "\($0.minFrameRate)-\($0.maxFrameRate)" })")
          })
        
        // Now try to select our high frame rate format
        if let format = device.formats
          .filter({ format in
            let dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            return dims.width == 1280 && dims.height == 720
          })
            .sorted(by: { format1, format2 in
              let maxFPS1 = format1.videoSupportedFrameRateRanges.map({ $0.maxFrameRate }).max() ?? 0
              let maxFPS2 = format2.videoSupportedFrameRateRanges.map({ $0.maxFrameRate }).max() ?? 0
              return maxFPS1 > maxFPS2
            })
              .first(where: { format in
                format.videoSupportedFrameRateRanges.contains(where: { $0.maxFrameRate >= 240 })
              }) {
          print("\nSelected format details:")
          print("Format: \(format)")
          print("Frame rates: \(format.videoSupportedFrameRateRanges.map { "\($0.minFrameRate)-\($0.maxFrameRate)" })")
          
          device.activeFormat = format
          device.activeVideoMinFrameDuration = CMTime(value: 1, timescale: 240)
          device.activeVideoMaxFrameDuration = CMTime(value: 1, timescale: 240)
          
          // Verify what was actually set
          print("\nVerification after setting:")
          print("Active format: \(device.activeFormat)")
          print("Min frame duration: \(device.activeVideoMinFrameDuration)")
          print("Max frame duration: \(device.activeVideoMaxFrameDuration)")
          print("Calculated FPS: \(1.0 / CMTimeGetSeconds(device.activeVideoMinFrameDuration))")
        }
        device.unlockForConfiguration()
      } catch {
        print("Error setting frame rate: \(error)")
      }
      
      do {
        let input = try AVCaptureDeviceInput(device: device)
        if captureSession.canAddInput(input) {
          captureSession.addInput(input)
        }
      } catch {
        print("Error adding video input: \(error)")
      }
      
      // Add video output
      let movieOutput = AVCaptureMovieFileOutput()
      
      // Set compression settings BEFORE adding output
      let compressionProperties: [String: Any] = [
        AVVideoAverageBitRateKey: 35_000_000,
        AVVideoMaxKeyFrameIntervalKey: 240,
        AVVideoAllowFrameReorderingKey: false,
        AVVideoExpectedSourceFrameRateKey: 240
      ]
      
      let outputSettings: [String: Any] = [
        AVVideoCodecKey: AVVideoCodecType.hevc,
        AVVideoCompressionPropertiesKey: compressionProperties
      ]
      
      if captureSession.canAddOutput(movieOutput) {
        captureSession.addOutput(movieOutput)
        
        if let connection = movieOutput.connection(with: .video) {
          movieOutput.setOutputSettings(outputSettings, for: connection)
          
          if connection.isVideoStabilizationSupported {
            connection.preferredVideoStabilizationMode = .off
          }
          
          if connection.isVideoOrientationSupported {
            // Get the current device orientation
            let orientation = UIDevice.current.orientation
            let videoOrientation: AVCaptureVideoOrientation
            
            switch orientation {
            case .portrait: videoOrientation = .portrait
            case .portraitUpsideDown: videoOrientation = .portraitUpsideDown
            case .landscapeLeft: videoOrientation = .landscapeRight  // Note: these are flipped
            case .landscapeRight: videoOrientation = .landscapeLeft  // Note: these are flipped
            default: videoOrientation = .portrait  // Default fallback
            }
            
            connection.videoOrientation = videoOrientation
          }
        }
        
        movieOutput.movieFragmentInterval = .invalid
        self.videoOutput = movieOutput
      }
      
      // Setup preview layer
      self.previewLayer = AVCaptureVideoPreviewLayer(session: captureSession)
      self.previewLayer?.videoGravity = .resizeAspectFill
      
      for handler in self.cameraPreviewLayerUpdateHandlers {
        handler(self.previewLayer!)
      }
      
      captureSession.commitConfiguration()
      captureSession.startRunning()
    }
  }
  
  // Function to provide preview layer
  func getPreviewLayer() -> AVCaptureVideoPreviewLayer? {
    return previewLayer
  }
  
  // Function to start recording
  func startRecording() {
    guard let videoOutput = videoOutput else { return }
    let outputPath = NSTemporaryDirectory() + "output.mp4"
    let outputURL = URL(fileURLWithPath: outputPath)
    
    // Remove previous file if exists
    try? FileManager.default.removeItem(at: outputURL)
    
    videoOutput.startRecording(to: outputURL, recordingDelegate: self)
  }
  
  // Function to stop recording
  func stopRecording() {
    videoOutput?.stopRecording()
  }
  
  // Function to copy video to pasteboard
  func copyToPasteboard() {
    guard let recordedVideoURL = recordedVideoURL else {
      print("No video recorded to copy.")
      return
    }
    
    do {
      let videoData = try Data(contentsOf: recordedVideoURL)
      UIPasteboard.general.setData(videoData, forPasteboardType: "public.movie")
      print("Video copied to pasteboard.")
    } catch {
      print("Failed to copy video to pasteboard: \(error)")
    }
  }
  
  // Function to share video via AirDrop
  func shareViaAirdrop() {
    guard let recordedVideoURL = recordedVideoURL else {
      print("No video recorded to share.")
      return
    }
    
    // Get the active window scene
    guard let windowScene = UIApplication.shared.connectedScenes.first as? UIWindowScene,
          let window = windowScene.keyWindow ?? windowScene.windows.first else {
      print("Unable to get window.")
      return
    }
    
    // Get the topmost view controller
    guard let topController = window.rootViewController?.topMostViewController() else {
      print("Unable to get top view controller.")
      return
    }
    
    let activityViewController = UIActivityViewController(
      activityItems: [recordedVideoURL],
      applicationActivities: nil
    )
    
    // For iPad: Set the source view for the popover
    if let popover = activityViewController.popoverPresentationController {
      popover.sourceView = window
      popover.sourceRect = CGRect(x: window.bounds.midX, y: window.bounds.midY, width: 0, height: 0)
      popover.permittedArrowDirections = []
    }
    
    topController.present(activityViewController, animated: true)
  }
}

extension CameraController: AVCaptureFileOutputRecordingDelegate {
  func fileOutput(_ output: AVCaptureFileOutput, didFinishRecordingTo outputFileURL: URL, from connections: [AVCaptureConnection], error: Error?) {
    if let error = error {
      print("Recording error: \(error)")
    } else {
      print("Video recording finished: \(outputFileURL)")
      
      // Get the AVAsset to check the actual recorded frame rate
      let asset = AVAsset(url: outputFileURL)
      if let track = asset.tracks(withMediaType: .video).first {
        print("Recorded video properties:")
        print("Nominal frame rate: \(track.nominalFrameRate)")
        print("Min frame duration: \(track.minFrameDuration)")
        print("Natural size: \(track.naturalSize)")
        
        // Get more detailed format description
        if let formatDescriptions = track.formatDescriptions as? [CMFormatDescription] {
          formatDescriptions.forEach { desc in
            print("Format description: \(desc)")
          }
        }
      }
      
      recordedVideoURL = outputFileURL
      finishedRecording = true
    }
  }
}
