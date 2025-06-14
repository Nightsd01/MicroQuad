//
//  BLEController.swift
//  QuadController
//
//  Created by Brad Hesse on 11/3/20.
//

import Foundation
import CoreBluetooth

protocol BLEControllerDelegate {
  func didConnect()
  func didDisconnect()
}

@objc protocol BLESensorCalibrationDelegate : AnyObject {
  func didGetCalibrationRequest(_ request : CalibrationRequest)
}

class BLEController : NSObject, CBCentralManagerDelegate, CBPeripheralDelegate, ObservableObject {
  static let serviceUUID = "ab0828b1-198e-4351-b779-901fa0e0371e"
  static let controlCharacteristicUUID = "4ac8a682-9736-4e5d-932b-e9b31405049c"
  static let telemetryCharacteristicUUID = "e35f992e-5e7c-11eb-ae93-0242ac130002"
  static let armCharacteristicUUID = "baf0bcca-634f-11eb-ae93-0242ac130002"
  static let resetCharacteristicUUID = "489d3a76-6fdf-11eb-9439-0242ac130002"
  static let motorDebugCharacteristicUUID = "dec9fad6-0cf9-11ec-82a8-0242ac130003"
  static let calibrateCharacteristicUUID = "498e876e-0dd2-11ec-82a8-0242ac130003"
  static let debugInfoCharacteristicUUID = "f0a0afee-0983-4691-adc5-02ee803f5418"
  static let pidTuningCharacteristicUUID = "58471750-7394-4659-bc69-09331eed05a3"
  static let maxUpdateFrequencySeconds = 0.1
  
  static let terminationString = "==TERMINATE=="
  
  var manager : CBCentralManager!
  var device : CBPeripheral?
  var connected = false
  var service : CBService?

  var controlCharacteristic : CBCharacteristic?
  var telemetryCharacteristic : CBCharacteristic?
  var armCharacteristic : CBCharacteristic?
  var resetCharacteristic : CBCharacteristic?
  var motorDebugCharacteristic : CBCharacteristic?
  var calibrationCharacteristic : CBCharacteristic?
  var debugInfoCharacteristic : CBCharacteristic?
  var pidTuningCharacteristic : CBCharacteristic?

  var lastUpdateTime : TimeInterval?
  var timer : Timer?
  var armStatus = false
  var debugData : Data?
  var debugDataExpectedBytes = 0
  var receivingGyroCalibrationData = false
  var debugPacketResponseTimer : Timer?
  @Published var receivingDebugData = false
  @Published var transferProgress = 0.0
  
  public var quadStatus = QuadStatus()
  @Published var bleStatus = "None"
  @Published var debugDataString : String?
  
  private var calibrationDelegates : [CalibrationType : BLESensorCalibrationDelegate] = [:]
  
  var delegate : BLEControllerDelegate? {
    didSet {
      updateDelegateState()
    }
  }
  
  public func updateArmStatusListener(_ listener : ArmStatusListener)
  {
    quadStatus.armStatusListener = listener
  }
  
  public func scan() {
    manager = CBCentralManager(delegate: nil, queue: nil)
    manager.delegate = self
  }
  
  public func sendControlUpdate(_ params : StickUpdateParams, endGesture : Bool) {
    if let lastTime = self.lastUpdateTime,
       !endGesture,
       Date().timeIntervalSince1970 - lastTime < BLEController.maxUpdateFrequencySeconds {
      if timer == nil {
        timer = Timer(timeInterval: BLEController.maxUpdateFrequencySeconds - (Date().timeIntervalSince1970 - lastTime), repeats: false, block: { [weak self] (updateTimer) in
          self?.timerFired(params)
        })
      }
    } else {
      sendUpdate(params)
    }
  }

  func recordDebugStateChange(record : Bool) {
    if let characteristic = debugInfoCharacteristic {
      device?.writeValue("record:\(record ? "1" : "0")".data(using: .utf8)!, for: characteristic, type:.withoutResponse)
    }
  }

  func getDebugInfo() {
    if let characteristic = debugInfoCharacteristic {
      device?.writeValue("request".data(using: .utf8)!, for: characteristic, type:.withoutResponse)
      device?.setNotifyValue(true, for: characteristic)
    }
  }
  
  func sendCalibrationUpdate(forSensorType type : CalibrationType, response : CalibrationResponse) {
    if let characteristic = calibrationCharacteristic {
      device?.writeValue("\(type.rawValue):\(response.rawValue)".data(using: .utf8)!, for: characteristic, type: .withoutResponse)
    }
  }

  func updateMotorDebugStatus(inProgress: Bool) {
    if let characteristic = motorDebugCharacteristic, armStatus == true {
      let motorDebugUpdate = "motordebug_enabled:\(inProgress ? 1 : 0)".data(using: .utf8)!
      device?.writeValue(motorDebugUpdate, for: characteristic, type: .withoutResponse)
    }
  }

  func manualMotorDebugSpeed(motor: Int, speed : Double) {
    assert(motor >= 1 && motor <= 4)
    if let characteristic = motorDebugCharacteristic, armStatus == true {
      let scaledValue = Int(255 * speed)
      let motorDebugUpdate = "motordebug_value:\(motor):\(scaledValue)".data(using: .utf8)!
      device?.writeValue(motorDebugUpdate, for: characteristic, type: .withoutResponse)
    }
  }
  
  func updateArmStatus(armed : Bool) {
    if let characteristic = armCharacteristic, armed != armStatus {
      armStatus = armed
      let armData = Data(bytes: &armStatus, count: MemoryLayout.size(ofValue: armStatus))
      device?.writeValue(armData, for: characteristic, type: .withoutResponse)
    }
  }
  
  func resetQuad() {
    var val = true
    if let characteristic = resetCharacteristic {
      let resetData = Data(bytes: &val, count: MemoryLayout.size(ofValue: val))
      device?.writeValue(resetData, for: characteristic, type: .withoutResponse)
    }
  }
  
  func timerFired(_ params : StickUpdateParams) {
    timer = nil
    sendUpdate(params)
  }
  
  private func sendUpdate(_ params : StickUpdateParams) {
    guard let characteristic = controlCharacteristic else {
      print("Unable to send update, not connected!")
      return
    }
    do {
      lastUpdateTime = Date().timeIntervalSince1970
      let data = try params.toData()
      device?.writeValue(data, for: characteristic, type: .withoutResponse)
    } catch {
      print("Encountered an error encoding JSON for control update: \(error)")
    }
  }
  
  // CBCentralManagerDelegate
  func centralManagerDidUpdateState(_ central: CBCentralManager) {
    updateDelegateState()
  }
  
  func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
    connected = true
    updateDelegateState()
    device = peripheral

    device?.delegate = self
    device?.discoverServices([CBUUID(string: BLEController.serviceUUID)])
  }
  
  func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
    connected = false
    updateDelegateState()
    device = nil
  }
  
  func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
    updateDelegateState()
    device = peripheral
    manager.stopScan()
    manager.connect(peripheral, options: nil)
  }
  
  func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
    updateDelegateState()
  }
  
  // CBPeripheralDelegate
  func peripheralDidUpdateName(_ peripheral: CBPeripheral) {
    updateDelegateState()
  }
  
  func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
    if let error = error {
      print(error)
      bleStatus = "Failure"
      return
    }
    
    guard let services = peripheral.services else {
      bleStatus = "Cannot use device"
      return
    }
    
    for service in services {
      if service.uuid.uuidString.lowercased() == BLEController.serviceUUID.lowercased() {
        peripheral.discoverCharacteristics(nil, for: service)
      }
    }
  }
  
  func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
    if let error = error {
      print(error)
      bleStatus = "Failure"
      return
    }
    
    guard let characteristics = service.characteristics else {
      bleStatus = "Failure"
      print("No characteristics for service")
      return;
    }
    
    for characteristic in characteristics {
      print("Got characteristic: \(characteristic.uuid.uuidString)")
      if characteristic.uuid.uuidString.lowercased() == BLEController.controlCharacteristicUUID.lowercased() {
        controlCharacteristic = characteristic
        bleStatus = "Ready"
        print("Registering quadcopter")
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.telemetryCharacteristicUUID.lowercased() {
        telemetryCharacteristic = characteristic
        device?.setNotifyValue(true, for: characteristic)
        bleStatus = "Telemetry ready"
        print("Telemetry ready")
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.armCharacteristicUUID.lowercased() {
        armCharacteristic = characteristic
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.resetCharacteristicUUID.lowercased() {
        resetCharacteristic = characteristic
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.motorDebugCharacteristicUUID.lowercased() {
        motorDebugCharacteristic = characteristic
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.calibrateCharacteristicUUID.lowercased() {
        calibrationCharacteristic = characteristic
        device?.setNotifyValue(true, for: characteristic)
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.debugInfoCharacteristicUUID.lowercased() {
        debugInfoCharacteristic = characteristic
        device?.setNotifyValue(true, for: characteristic)
      } else if characteristic.uuid.uuidString.lowercased() == BLEController.pidTuningCharacteristicUUID.lowercased() {
        pidTuningCharacteristic = characteristic
      }
    }
  }

  func processDebugData(_ data : Data) {
    print("Got debug data with MD5 = \(data.md5Hash) total bytes = \(data.count)")
    // 8 bytes: timestamp
    // 24 bytes: yaw pitch roll[3]
    // 24 bytes: accel raw[3]
    // 24 bytes: gyro raw[3]
    // 24 bytes: accel filtered[3]
    // 24 bytes: gyro filtered[3]
    // 24 bytes: angle PID outputs[3]
    // 24 bytes: rate PID outputs[3]
    // 32 bytes: motor values[4]
    // 32 bytes: mag raw values[4]
    // 32 bytes: mag post-sof/hard matrix calibration values[4]
    // 32 bytes: mag post motor-mag calibration values[4]
    // 8 bytes: throttle
    // 8 bytes: voltage
    // 24 bytes: set points[3]
    // 32 bytes: EKF quaternion
    // 24 bytes: EKF yaw pitch roll
    // 8 bytes: EKF altitude
    // 8 bytes: EKF vertical velocity
    // 8 bytes: barometer relative altitude
    // 8 bytes: laser rangefinder relative altitude
    var result = ""
    let valuesPerPacket = 54
    let byteCount = data.count / 8
    for i in 0..<byteCount {
      let value = data.withUnsafeBytes { buffer in
        return buffer.load(fromByteOffset: i * 8, as: Double.self)
      }
      if (i == (valuesPerPacket - 1) || (i + 1) % valuesPerPacket == 0) && i > 0 {
        result += String(format: "%.2f", value) + "\n"
      } else {
        result += String(format: "%.2f", value) + ", "
      }
    }

    debugDataString = result
  }

  func debugUpdate(_ characteristic : CBCharacteristic) {
    guard let data = characteristic.value else {
      print("Got invalid packet")
      return
    }

    if !receivingDebugData {
      guard let dataString = String(data: data, encoding: .utf8) else {
        print("Got invalid first packet (1)")
        return
      }
      let components = dataString.components(separatedBy: ":")
      guard components.count > 1,
            let num = Int(components[1]) else {
        print("Got invalid first packet (2)")
        return
      }
      debugDataExpectedBytes = num
      debugData = Data()
      if (!receivingDebugData) {
        receivingDebugData = true
      }
    } else {
      guard debugData != nil else {
        fatalError("debug data was not initialized")
      }
      if receivingDebugData && String(data: data, encoding: .utf8) == BLEController.terminationString {
        processDebugData(debugData!)
        receivingDebugData = false
        return
      }
      
      transferProgress = Double(debugData!.count) / Double(debugDataExpectedBytes)
      debugData = debugData! + data
      device?.setNotifyValue(true, for: characteristic)
      debugPacketResponseTimer?.invalidate()
      debugPacketResponseTimer = Timer.scheduledTimer(withTimeInterval: 0.3, repeats: false, block: { [weak self] timer in
        self?.device?.setNotifyValue(true, for: characteristic)
        print("Timer notified")
      })
      print("Debug data length: \(debugData!.count), expecting: \(debugDataExpectedBytes)")
      if (debugData!.count == debugDataExpectedBytes) {
        print("Finished data transmission")
        processDebugData(debugData!)
        receivingDebugData = false
      }
      
    }
  }
  
  func addCalibrationDelegate(forSensorType type : CalibrationType, calibrationDelegate : BLESensorCalibrationDelegate) {
    calibrationDelegates[type] = calibrationDelegate
  }

  // we unfortunately transmit 10 bytes at a time due to iOS BLE packet size limits....
  func calibrationUpdate(_ characteristic : CBCharacteristic) {
    guard let data = characteristic.value else {
      print("Got invalid calibration packet")
      return
    }
    
    if let str = String(data: data, encoding: .utf8) {
      let components = str.components(separatedBy: ":")
      guard components.count == 2,
            let sensorTypeCase = UInt8(components[0]),
            let sensorType = CalibrationType(rawValue: sensorTypeCase),
            let updateCase = UInt8(components[1]),
            let request = CalibrationRequest(rawValue: updateCase) else {
        print("Got invalid accel+gyro calibration update")
        return
      }
      
      calibrationDelegates[sensorType]?.didGetCalibrationRequest(request)
    } else {
      print("Unable to handle calibration update")
    }
  }
  
  func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
    quadStatus.rssi = RSSI.doubleValue
  }
  
  func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
    if characteristic.uuid.uuidString.lowercased() == BLEController.debugInfoCharacteristicUUID.lowercased() {
      debugUpdate(characteristic)
      return
    } else if characteristic.uuid.uuidString.lowercased() == BLEController.calibrateCharacteristicUUID.lowercased() {
      calibrationUpdate(characteristic)
      return
    }

    guard characteristic.uuid.uuidString.lowercased() == BLEController.telemetryCharacteristicUUID.lowercased() else {
      print("Unrecognized characteristic wrote value")
      return
    }
    
    guard let data = characteristic.value else {
      print("Received no telemetry data from device")
      return
    }
    
    quadStatus.updateTelemetry(withData: data)
  }
  
  func updatePIDValues(axis : ControlAxis, type : PIDType, values : inout PIDValues)
  {
    guard let characteristic = pidTuningCharacteristic else {
      print("Error updating PIDs: No characteristic")
      return
    }
    var controlAxis = axis.rawValue
    var pidType = type.rawValue
    var buffer = Data()
    withUnsafeBytes(of: &controlAxis) {
      buffer.append(contentsOf: $0)
    }
    withUnsafeBytes(of: &pidType) {
      buffer.append(contentsOf: $0)
    }
    withUnsafeBytes(of: &values) {
      buffer.append(contentsOf: $0)
    }
    device?.writeValue(buffer, for: characteristic, type: .withoutResponse)
  }
  
  func updatePID(axis : ControlAxis, type : PIDType, gains : PIDGains)
  {
    var values : PIDValues
    switch (axis) {
    case .Yaw:
      values = gains.yawGains
    case .Pitch:
      values = gains.pitchGains
    case .Roll:
      values = gains.rollGains
    default:
      fatalError("Unsupported axis \(axis)")
    }
    
    updatePIDValues(axis: axis, type: type, values: &values)
  }
  
  private func updateDelegateState() {
    if (connected) {
      bleStatus = "Connected to " + (device?.name ?? "Unknown")
      return
    }

    switch manager.state {
      case .poweredOff:
        bleStatus = "Disconnected"
        break
      case .poweredOn:
        bleStatus = "Disconnected"
        manager.scanForPeripherals(withServices: [CBUUID(string: BLEController.serviceUUID)], options: nil)
        break
      case .unknown:
        bleStatus = "Unknown"
        break
      case .unauthorized:
        bleStatus = "Unauthorized"
        break
      case .resetting:
        bleStatus = "Resetting"
        break
      case .unsupported:
        bleStatus = "Unsupported"
        break
      @unknown default:
        bleStatus = "Unsupported State"
        break
    }
  }
}
