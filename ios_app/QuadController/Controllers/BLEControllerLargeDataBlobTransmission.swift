//
//  BLEControllerPacketizer.swift
//  QuadController
//
//  Created by Brad Hesse on 7/31/25.
//

import Foundation
import CoreBluetooth

// MARK: - L2CAP Header Definition

// Swift representation of the L2CAP blob header used by firmware
// Layout: [type: 1 byte][payload_size_bytes: 4 bytes little-endian]
struct BLELargeDataBlobHeader {
  let type: BLELargeDataBlobType
  let payloadSizeBytes: UInt32

  static let byteCount = 5

  init?(from data: Data) {
    guard data.count >= BLELargeDataBlobHeader.byteCount else { return nil }
    let typeByte = data[data.startIndex]
    guard let t = BLELargeDataBlobType(rawValue: typeByte) else { return nil }
    type = t
    let b1 = UInt32(data[data.startIndex &+ 1])
    let b2 = UInt32(data[data.startIndex &+ 2])
    let b3 = UInt32(data[data.startIndex &+ 3])
    let b4 = UInt32(data[data.startIndex &+ 4])
    payloadSizeBytes = (b1) | (b2 << 8) | (b3 << 16) | (b4 << 24)
  }

  init(type: BLELargeDataBlobType, payloadSizeBytes: UInt32) {
    self.type = type
    self.payloadSizeBytes = payloadSizeBytes
  }

  func toData() -> Data {
    var d = Data(count: BLELargeDataBlobHeader.byteCount)
    d[0] = type.rawValue
    var sizeLE = payloadSizeBytes.littleEndian
    withUnsafeBytes(of: &sizeLE) { d.replaceSubrange(1..<5, with: $0) }
    return d
  }
}

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
  
  // RX reassembly buffer/state
  private var rxBuffer = Data()
  private var rxExpectedPayloadBytes: UInt32? = nil
  
  // Callback for fully reassembled blobs
  var onReceiveBlob: ((BLELargeDataBlobType, Data) -> Void)?
  
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
    outputStream?.remove(from: .main, forMode: .default)
    inputStream?.remove(from: .main, forMode: .default)
    
    // Clean up
    outputStream = nil
    inputStream = nil
    rxBuffer.removeAll(keepingCapacity: false)
    rxExpectedPayloadBytes = nil
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
    
    // If a previous transmission is in-flight and streams got into a bad state, attempt a light reset
    if outputStream == nil || inputStream == nil {
      print("L2CAP streams nil; attempting channel reconnect for PSM: \(psm)")
      completion(NSError(domain: "BLEController", code: 6, userInfo: [NSLocalizedDescriptionKey: "Streams not available"]))
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
    
    // Build header: [type:1][payload_size:4 LE] is handled by caller. Here we expect 'data' to already include type byte first.
    // So we compute payload size as data.count - 1 (exclude type byte) and prefix header accordingly.
    guard let typeByte = data.first, let blobType = BLELargeDataBlobType(rawValue: typeByte) else {
      transmissionCompletion?(NSError(domain: "BLEController", code: 2, userInfo: [NSLocalizedDescriptionKey: "Invalid blob type for transmission"]))
      return
    }
    let payloadSize = UInt32(max(0, data.count - 1))
    let header = BLELargeDataBlobHeader(type: blobType, payloadSizeBytes: payloadSize)
    var fullData = header.toData()
    fullData.append(data.dropFirst())
    
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
    
    var bytesRemaining = data.count - bytesSent
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
    
    // Write as much as possible while the stream has space
    data.withUnsafeBytes { rawBuf in
      guard let base = rawBuf.bindMemory(to: UInt8.self).baseAddress else { return }
      while outputStream.hasSpaceAvailable && bytesRemaining > 0 {
        let maxChunk = min(bytesRemaining, 16384)
        let ptr = base.advanced(by: bytesSent)
        let written = outputStream.write(ptr, maxLength: maxChunk)
        if written > 0 {
          bytesSent += written
          bytesRemaining -= written
          let progress = Double(bytesSent) / Double(data.count)
          print("L2CAP wrote \(written) bytes, total: \(bytesSent)/\(data.count) (\(String(format: "%.1f", progress * 100))%)")
        } else if written == 0 {
          break
        } else {
          transmissionCompletion?(outputStream.streamError ?? NSError(domain: "BLEController", code: 2, userInfo: [NSLocalizedDescriptionKey: "Stream write error"]))
          transmissionCompletion = nil
          currentTransmissionData = nil
          return
        }
      }
    }
    
    // If there is still data remaining, we'll resume on next hasSpaceAvailable event
  }
}

// MARK: - StreamDelegate

extension L2CAPChannelManager: StreamDelegate {
  func stream(_ aStream: Stream, handle eventCode: Stream.Event) {
    switch eventCode {
    case .hasBytesAvailable:
      if aStream == inputStream { readAvailableBytes() }
      
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
  
  // MARK: - RX processing
  private func readAvailableBytes() {
    guard let inputStream = inputStream else { return }
    var tempBuffer = [UInt8](repeating: 0, count: 1024)
    while inputStream.hasBytesAvailable {
      let readBytes = inputStream.read(&tempBuffer, maxLength: tempBuffer.count)
      if readBytes > 0 {
        rxBuffer.append(contentsOf: tempBuffer.prefix(readBytes))
      } else if readBytes < 0 {
        print("L2CAP input stream read error: \(inputStream.streamError?.localizedDescription ?? "Unknown")")
        break
      } else {
        break
      }
    }
    processRxBuffer()
  }
  
  private func processRxBuffer() {
    // We may have multiple blobs back-to-back; loop until we cannot parse more
    while true {
      // Need header first (5 bytes)
      if rxExpectedPayloadBytes == nil {
        guard rxBuffer.count >= BLELargeDataBlobHeader.byteCount else { return }
        guard let header = BLELargeDataBlobHeader(from: rxBuffer) else {
          print("L2CAP RX: invalid header")
          return
        }
        rxExpectedPayloadBytes = header.payloadSizeBytes + 1 // include an injected type byte
        // Remove header
        rxBuffer.removeFirst(BLELargeDataBlobHeader.byteCount)
        // Inject type as leading byte so downstream stays the same
        rxBuffer.insert(header.type.rawValue, at: rxBuffer.startIndex)
      }
      guard let expected = rxExpectedPayloadBytes else { return }
      guard rxBuffer.count >= Int(expected) else { return }
      // Extract payload
      let payload = rxBuffer.prefix(Int(expected))
      rxBuffer.removeFirst(Int(expected))
      rxExpectedPayloadBytes = nil
      
      // First byte is type
      guard let first = payload.first,
            let blobType = BLELargeDataBlobType(rawValue: first) else {
        let bad = payload.first ?? 0
        print("L2CAP RX: invalid blob type \(bad)")
        continue
      }
      let dataOnly = payload.dropFirst()
      onReceiveBlob?(blobType, Data(dataOnly))
      // Continue; there may be more data in buffer
    }
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
    
    // PSM is typically 2 bytes, little-endian (avoid unaligned loads)
    let lo = UInt16(data.first ?? 0)
    let hi = data.count > 1 ? UInt16(data[1]) : 0
    let psm = (hi << 8) | lo
    
    print("Read L2CAP PSM from characteristic: \(psm)")
    
    // Open the large-data channel with the discovered PSM
    device?.openL2CAPChannel(CBL2CAPPSM(psm))
    // Open the telemetry channel on adjacent PSM (firmware uses 0x0081)
    let telemetryPSM = CBL2CAPPSM(0x81)
    if telemetryManager == nil { telemetryManager = L2CAPChannelManager(psm: telemetryPSM) }
    device?.openL2CAPChannel(telemetryPSM)
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
    
    print("L2CAP channel opened successfully on PSM: \(channel.psm)")
    if let lm = l2capManager, channel.psm == lm.psm {
      lm.connect(channel: channel)
      lm.onReceiveBlob = { [weak self] type, data in
        guard let self = self else { return }
        if type == .TelemetryData { self.handleTelemetryBatch(data) }
        else if type == .PIDConfigurationData { self.handlePIDConfiguration(data) }
      }
    } else {
      if telemetryManager == nil { telemetryManager = L2CAPChannelManager(psm: channel.psm) }
      telemetryManager?.connect(channel: channel)
      telemetryManager?.onReceiveBlob = { [weak self] type, data in
        guard let self = self else { return }
        if type == .TelemetryData { self.handleTelemetryBatch(data) }
      }
    }
    l2capChannelOpenCompletion?(nil)
    l2capChannelOpenCompletion = nil
  }


  // MARK: - Telemetry Batch Parsing
  // Batch frame format: [eventId: u8][size: u16 LE][payload: size bytes] x N
  fileprivate func handleTelemetryBatch(_ batch: Data) {
    var cursor = 0
    let bytes = [UInt8](batch)
    while cursor < bytes.count {
      // Need at least 3 bytes for header
      guard cursor + 3 <= bytes.count else { break }
      let eventId = bytes[cursor]
      let sizeLE = UInt16(bytes[cursor + 1]) | (UInt16(bytes[cursor + 2]) << 8)
      cursor += 3
      let size = Int(sizeLE)
      guard cursor + size <= bytes.count else { break }
      let payload = Data(bytes[cursor..<(cursor + size)])
      cursor += size
      // Reconstruct legacy telemetry packet: [eventId][payload]
      var packet = Data()
      packet.append(eventId)
      packet.append(payload)
      self.quadStatus.updateTelemetry(withData: packet)
    }
  }
  
  // MARK: - PID Configuration Data Parsing
  // Format: 21 floats (84 bytes total), all little-endian
  // [0-35]: angleGains (yaw P,I,D, pitch P,I,D, roll P,I,D) = 9 floats
  // [36-71]: rateGains (yaw P,I,D, pitch P,I,D, roll P,I,D) = 9 floats  
  // [72-83]: verticalVelocityGains (P,I,D) = 3 floats
  fileprivate func handlePIDConfiguration(_ data: Data) {
    guard data.count == 84 else { // 21 floats * 4 bytes
      print("Invalid PID configuration data size: \(data.count), expected 84")
      return
    }
    
    guard let pidController = self.pidController else {
      print("PID Controller not initialized")
      return
    }
    
    // Helper to read a float at given offset
    func readFloat(at offset: Int) -> Float32 {
      let bytes = data.subdata(in: offset..<offset+4)
      return bytes.withUnsafeBytes { $0.load(as: Float32.self) }
    }
    
    var offset = 0
    
    // Read angle gains
    let angleYaw = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    let anglePitch = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    let angleRoll = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    // Read rate gains
    let rateYaw = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    let ratePitch = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    let rateRoll = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    offset += 12
    
    // Read vertical velocity gains
    let verticalGains = PIDValues(
      proportional: readFloat(at: offset),
      integral: readFloat(at: offset + 4),
      derivative: readFloat(at: offset + 8)
    )
    
    // Update the PID controller with received values
    pidController.gains = PIDsContainer(
      angleValues: PIDGains(
        yawGains: angleYaw,
        pitchGains: anglePitch,
        rollGains: angleRoll
      ),
      rateValues: PIDGains(
        yawGains: rateYaw,
        pitchGains: ratePitch,
        rollGains: rateRoll
      ),
      verticalGains: verticalGains
    )
    
    print("PID configuration received from ESP32")
  }
}
