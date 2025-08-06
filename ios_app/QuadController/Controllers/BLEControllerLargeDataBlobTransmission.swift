//
//  BLEControllerPacketizer.swift
//  QuadController
//
//  Created by Brad Hesse on 7/31/25.
//

import Foundation
import CoreBluetooth

// MARK: - L2CAP Channel Management

/// Represents the state of an L2CAP channel
enum L2CAPChannelState {
  case disconnected
  case connecting
  case connected
  case disconnecting
}

/// Manages L2CAP channel operations and data transmission
class L2CAPChannelManager: NSObject {
  private var channel: CBL2CAPChannel?
  var channelState: L2CAPChannelState = .disconnected { // Made accessible
    didSet {
      print("L2CAP channel state changed to: \(channelState)")
    }
  }
  private var pendingData: Data?
  private var transmissionCompletion: ((Error?) -> Void)?
  private var outputStream: OutputStream?
  private var inputStream: InputStream?
  
  // Data being transmitted
  private var currentTransmissionData: Data?
  
  // Statistics
  private var bytesSent: Int = 0
  private var totalBytesToSend: Int = 0
  private var transmissionStartTime: Date?
  
  /// PSM (Protocol/Service Multiplexer) for our custom L2CAP service
  /// iOS dynamically assigns PSMs in the range 0x0080-0x00FF
  let psm: CBL2CAPPSM
  
  init(psm: CBL2CAPPSM) {
    self.psm = psm
  }
  
  func connect(channel: CBL2CAPChannel) {
    self.channel = channel
    self.channelState = .connected
    self.outputStream = channel.outputStream
    self.inputStream = channel.inputStream
    
    // Configure streams
    outputStream?.delegate = self
    inputStream?.delegate = self
    
    // Schedule streams on the main run loop
    outputStream?.schedule(in: .main, forMode: .default)
    inputStream?.schedule(in: .main, forMode: .default)
    
    // Open streams
    outputStream?.open()
    inputStream?.open()
    
    print("L2CAP channel connected with PSM: \(psm)")
    
    // If we have pending data, start transmission
    if let data = pendingData {
      startTransmission(data: data)
    }
  }
  
  func disconnect() {
    channelState = .disconnecting
    
    // Close streams
    outputStream?.close()
    inputStream?.close()
    
    // Remove from run loop
    outputStream?.remove(from: .main, forMode: .default)
    inputStream?.remove(from: .main, forMode: .default)
    
    // Clean up
    outputStream = nil
    inputStream = nil
    channel = nil
    channelState = .disconnected
    
    print("L2CAP channel disconnected")
  }
  
  func transmit(data: Data, completion: @escaping (Error?) -> Void) {
    guard channelState == .connected else {
      pendingData = data
      transmissionCompletion = completion
      print("L2CAP channel not connected, queuing data for transmission")
      return
    }
    
    transmissionCompletion = completion
    startTransmission(data: data)
  }
  
  private func startTransmission(data: Data) {
    pendingData = nil
    bytesSent = 0
    totalBytesToSend = data.count
    transmissionStartTime = Date()
    
    print("Starting L2CAP transmission of \(data.count) bytes")
    
    // Write data header first (4 bytes for size of entire payload including type byte)
    var dataSize = UInt32(data.count).littleEndian
    let headerData = Data(bytes: &dataSize, count: 4)
    
    // Combine header and data
    var fullData = headerData
    fullData.append(data)
    
    // Store the full data for transmission
    currentTransmissionData = fullData
    totalBytesToSend = fullData.count
    
    // Start writing
    writeDataToStream()
  }
  
  private func writeDataToStream() {
    guard let outputStream = outputStream,
          let data = currentTransmissionData else {
      transmissionCompletion?(NSError(domain: "BLEController", code: 1, userInfo: [NSLocalizedDescriptionKey: "Output stream or data not available"]))
      return
    }
    
    let bytesRemaining = data.count - bytesSent
    if bytesRemaining <= 0 {
      // Transmission complete
      let duration = Date().timeIntervalSince(transmissionStartTime ?? Date())
      let payloadSize = data.count - 4  // Excluding 4-byte header
      let throughput = Double(payloadSize) / duration / 1024.0 // KB/s (excluding header)
      print("L2CAP transmission complete: \(payloadSize) bytes payload in \(String(format: "%.2f", duration))s (\(String(format: "%.1f", throughput)) KB/s)")
      
      currentTransmissionData = nil
      transmissionCompletion?(nil)
      transmissionCompletion = nil
      return
    }
    
    // Check if stream has space available
    guard outputStream.hasSpaceAvailable else {
      // Will continue when stream has space (handled in StreamDelegate)
      return
    }
    
    // Write data in chunks
    let chunkSize = min(bytesRemaining, 512) // L2CAP MTU is typically 512 bytes
    let chunk = data.subdata(in: bytesSent..<(bytesSent + chunkSize))
    
    chunk.withUnsafeBytes { bytes in
      let written = outputStream.write(bytes.bindMemory(to: UInt8.self).baseAddress!, maxLength: chunkSize)
      if written > 0 {
        bytesSent += written
        let progress = Double(bytesSent) / Double(data.count)
        print("L2CAP wrote \(written) bytes, total: \(bytesSent)/\(data.count) (\(String(format: "%.1f", progress * 100))%)")
        
        // Continue writing if there's more data
        if bytesSent < data.count {
          // Schedule next write on the next run loop iteration
          DispatchQueue.main.async { [weak self] in
            self?.writeDataToStream()
          }
        }
      } else if written < 0 {
        // Error occurred
        transmissionCompletion?(outputStream.streamError ?? NSError(domain: "BLEController", code: 2, userInfo: [NSLocalizedDescriptionKey: "Stream write error"]))
        transmissionCompletion = nil
        currentTransmissionData = nil
      }
    }
  }
}

// MARK: - StreamDelegate

extension L2CAPChannelManager: StreamDelegate {
  func stream(_ aStream: Stream, handle eventCode: Stream.Event) {
    switch eventCode {
    case .hasBytesAvailable:
      // Handle incoming data (for future RX implementation)
      if aStream == inputStream {
        handleIncomingData()
      }
      
    case .hasSpaceAvailable:
      // Continue writing if we have more data
      if aStream == outputStream && currentTransmissionData != nil {
        print("L2CAP output stream has space available, continuing transmission")
        writeDataToStream()
      }
      
    case .errorOccurred:
      print("L2CAP stream error: \(aStream.streamError?.localizedDescription ?? "Unknown error")")
      transmissionCompletion?(aStream.streamError)
      transmissionCompletion = nil
      currentTransmissionData = nil
      
    case .endEncountered:
      print("L2CAP stream ended")
      if let data = currentTransmissionData, bytesSent < data.count {
        transmissionCompletion?(NSError(domain: "BLEController", code: 3, userInfo: [NSLocalizedDescriptionKey: "Stream ended unexpectedly"]))
        transmissionCompletion = nil
        currentTransmissionData = nil
      }
      
    default:
      break
    }
  }
  
  private func handleIncomingData() {
    // Placeholder for future RX implementation
    // This will handle receiving large data blobs from the ESP32
  }
}

// MARK: - BLEController Extension

extension BLEController {
  
  /// Transmits a large data blob using L2CAP Connection-oriented Channels
  /// - Parameters:
  ///   - type: The type of data being transmitted (will be prefixed as first byte)
  ///   - data: The data to transmit
  ///   - completion: Completion handler called when transmission completes or fails
  func transmitLargeDataBlob(_ type : BLELargeDataBlobType, _ data: Data, completion: @escaping (Error?) -> Void) {
    guard device != nil else {
      completion(NSError(domain: "BLEController", code: 100, userInfo: [NSLocalizedDescriptionKey: "No device connected"]))
      return
    }
    
    guard data.count > 0 else {
      completion(NSError(domain: "BLEController", code: 101, userInfo: [NSLocalizedDescriptionKey: "Empty data provided"]))
      return
    }
    
    // Prefix the data with the type byte
    var typedData = Data()
    typedData.append(UInt8(type.rawValue))
    typedData.append(data)
    
    print("Initiating L2CAP transmission for \(typedData.count) bytes (type: \(type.rawValue), data: \(data.count) bytes)")
    
    // Initialize L2CAP manager if needed
    if l2capManager == nil {
      l2capManager = L2CAPChannelManager(psm: 0x80) // Default PSM, will be updated
    }
    
    // Check if we already have an L2CAP channel open
    if l2capManager?.channelState == .connected {
      // Use existing channel
      l2capManager?.transmit(data: typedData, completion: completion)
    } else {
      // Need to open L2CAP channel first
      openL2CAPChannel { error in
        if let error = error {
          completion(error)
        } else {
          self.l2capManager?.transmit(data: typedData, completion: completion)
        }
      }
    }
  }
  
  /// Opens an L2CAP channel to the connected device
  private func openL2CAPChannel(completion: @escaping (Error?) -> Void) {
    guard let device = device else {
      completion(NSError(domain: "BLEController", code: 102, userInfo: [NSLocalizedDescriptionKey: "No device connected"]))
      return
    }
    
    // First, we need to read the PSM from the characteristic
    guard let psmCharacteristic = l2capPSMCharacteristic else {
      completion(NSError(domain: "BLEController", code: 104, userInfo: [NSLocalizedDescriptionKey: "L2CAP PSM characteristic not found"]))
      return
    }
    
    // Read the PSM value
    device.readValue(for: psmCharacteristic)
    
    // Store completion for when PSM is read and channel opens
    l2capChannelOpenCompletion = completion
  }
  
  /// Handles the PSM value read from the characteristic
  func handlePSMValue(_ data: Data) {
    guard data.count >= 2 else {
      l2capChannelOpenCompletion?(NSError(domain: "BLEController", code: 105, userInfo: [NSLocalizedDescriptionKey: "Invalid PSM data"]))
      l2capChannelOpenCompletion = nil
      return
    }
    
    // PSM is typically 2 bytes, little-endian
    let psm = data.withUnsafeBytes { $0.load(as: UInt16.self) }
    
    print("Read L2CAP PSM from characteristic: \(psm)")
    
    // Open the L2CAP channel with the discovered PSM
    device?.openL2CAPChannel(CBL2CAPPSM(psm))
  }
}

// MARK: - CBPeripheralDelegate L2CAP Extensions

extension BLEController {
  
  // This method is called when an L2CAP channel is opened
  func peripheral(_ peripheral: CBPeripheral, didOpen channel: CBL2CAPChannel?, error: Error?) {
    if let error = error {
      print("Failed to open L2CAP channel: \(error)")
      l2capChannelOpenCompletion?(error)
      l2capChannelOpenCompletion = nil
      return
    }
    
    guard let channel = channel else {
      let error = NSError(domain: "BLEController", code: 103, userInfo: [NSLocalizedDescriptionKey: "L2CAP channel is nil"])
      l2capChannelOpenCompletion?(error)
      l2capChannelOpenCompletion = nil
      return
    }
    
    print("L2CAP channel opened successfully")
    l2capManager?.connect(channel: channel)
    l2capChannelOpenCompletion?(nil)
    l2capChannelOpenCompletion = nil
  }
}

// MARK: - Usage Example

/*
 Usage example for transmitting EPO data:
 
 ```swift
 // In your GPS controller or main app
 if let epoData = GPSController.shared.getEPODataForL96() {
 bleController.transmitLargeDataBlob(epoData) { error in
 if let error = error {
 print("Failed to transmit EPO data: \(error)")
 } else {
 print("EPO data transmitted successfully")
 }
 }
 }
 ```
 
 ESP32 Implementation Notes:
 1. The ESP32 must advertise an L2CAP PSM in a GATT characteristic
 2. The ESP32 must listen for incoming L2CAP connections on that PSM
 3. The data format includes a 4-byte header with the data size (little-endian)
 4. The ESP32 should be prepared to receive data in chunks
 
 Future RX Implementation:
 - Add a delegate protocol for receiving data
 - Implement data reassembly for incoming chunks
 - Add flow control mechanisms
 - Support bidirectional communication
 */

