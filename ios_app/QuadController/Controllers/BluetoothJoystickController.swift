//
//  BluetoothJoystickController.swift
//  QuadController
//
//  Created by Brad Hesse on 5/8/25.
//

import Foundation
import SwiftUI // For ObservableObject and Published
import GameController // For all GameController framework interactions

/// A struct to hold continuous input values from the game controller's analog sticks.
/// This struct MUST be initialized with explicit stick values.
public struct AnalogInputState: Equatable {
  public var leftStick: XY
  public var rightStick: XY
  
  public init(leftStick: XY, rightStick: XY) {
    self.leftStick = leftStick
    self.rightStick = rightStick
  }
}

// MARK: - BluetoothStickController Class

class BluetoothStickController: ObservableObject {
  
  var bluetoothController : BLEController!
  
  // MARK: - Published Properties for SwiftUI
  /// The current analog input values for the left stick. Nil if not connected or no input.
  @Published public var leftStickValues: XY? = nil
  
  /// The current analog input values for the right stick. Nil if not connected or no input.
  @Published public var rightStickValues: XY? = nil
  
  /// True if Button A is currently pressed.
  @Published public var isButtonAPressed: Bool = false
  
  /// True if a compatible joystick is connected.
  @Published public var isConnected: Bool = false
  
  /// The name of the connected controller, if available.
  @Published public var connectedControllerName: String? = nil
  
  public weak var armStatusListener : ArmStatusListener?
  
  
  // MARK: - Private Properties
  private var connectedController: GCController? = nil
  private var notificationObservers: [NSObjectProtocol] = []
  private var deadZone: Float
  private var controlUpdateTimer: Timer? = nil
  
  // MARK: - Initialization
  public init(deadZone: Float = 0.15) {
    self.deadZone = deadZone
    print("BluetoothStickController: Initializing with deadzone \(deadZone)")
    setupConnectionObservers()
    checkInitialConnection() // Check for already connected controllers
    startDiscovery()         // Start looking for new controllers
  }
  
  deinit {
    print("BluetoothStickController: Deinitializing.")
    stopControlUpdateTimer()
    stopDiscovery()
    notificationObservers.forEach(NotificationCenter.default.removeObserver)
    notificationObservers.removeAll()
  }
  
  // MARK: - Public Controller Management
  
  public func provideBLEController(_ bluetoothController : BLEController)
  {
    self.bluetoothController = bluetoothController
  }
  
  /// Starts scanning for wireless game controllers.
  public func startDiscovery() {
    GCController.startWirelessControllerDiscovery {
      print("BluetoothStickController: Wireless controller discovery started/completed.")
    }
  }
  
  /// Stops scanning for wireless game controllers.
  public func stopDiscovery() {
    GCController.stopWirelessControllerDiscovery()
    print("BluetoothStickController: Wireless controller discovery stopped.")
  }
  
  // MARK: - Connection Management
  
  private func startControlUpdateTimer() {
    stopControlUpdateTimer() // Ensure no duplicate timers
    
    // Create a timer that fires 10 times per second (every 0.1 seconds)
    controlUpdateTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in
      self?.sendPeriodicControlUpdate()
    }
    print("BluetoothStickController: Started control update timer (10Hz)")
  }
  
  private func stopControlUpdateTimer() {
    controlUpdateTimer?.invalidate()
    controlUpdateTimer = nil
    print("BluetoothStickController: Stopped control update timer")
  }
  
  private func sendPeriodicControlUpdate() {
    // Only send if we have a BLE controller and are connected
    guard let bleController = bluetoothController, isConnected else { return }
    
    let left = self.leftStickValues ?? XY(x: 0.0, y: 0.0)
    let right = self.rightStickValues ?? XY(x: 0.0, y: 0.0)
    
    // Convert stick values to the expected range for the drone
    let rescaledStickParams = StickUpdateParams(
      throttle: (((Double(left.y) / 2.0) + 1.0) * 255.0) - 127.5,
      yaw: (((Double(left.x) / 2.0) + 1.0) * 255.0) - 127.5,
      pitch: (((Double(right.y) / 2.0) + 1.0) * 255.0) - 127.5,
      roll: (((Double(right.x) / 2.0) + 1.0) * 255.0) - 127.5)
    
    bleController.sendControlUpdate(rescaledStickParams, endGesture: false)
  }
  
  private func setupConnectionObservers() {
    // Observer for when a controller connects
    let connectObserver = NotificationCenter.default.addObserver(
      forName: .GCControllerDidConnect,
      object: nil,
      queue: .main // Operations will be on the main queue
    ) { [weak self] notification in
      guard let self = self, let controller = notification.object as? GCController else { return }
      print("BluetoothStickController: GCControllerDidConnect notification - \(controller.vendorName ?? "Unknown Vendor")")
      self.connectToController(controller)
    }
    notificationObservers.append(connectObserver)
    
    // Observer for when a controller disconnects
    let disconnectObserver = NotificationCenter.default.addObserver(
      forName: .GCControllerDidDisconnect,
      object: nil,
      queue: .main // Operations will be on the main queue
    ) { [weak self] notification in
      guard let self = self, let disconnectedController = notification.object as? GCController else { return }
      print("BluetoothStickController: GCControllerDidDisconnect notification - \(disconnectedController.vendorName ?? "Unknown Vendor")")
      if self.connectedController == disconnectedController {
        self.handleDisconnection()
      } else {
        // A different controller disconnected, check if our current one is still valid
        self.checkInitialConnection()
      }
    }
    notificationObservers.append(disconnectObserver)
  }
  
  private func handleDisconnection() {
    DispatchQueue.main.async {
      self.stopControlUpdateTimer() // Stop sending updates when disconnected
      self.connectedController = nil
      self.isConnected = false
      self.connectedControllerName = nil
      self.leftStickValues = nil
      self.rightStickValues = nil
      self.isButtonAPressed = false
      print("BluetoothStickController: Processed disconnection.")
    }
  }
  
  /// Checks for already connected controllers at startup or after a potential change.
  private func checkInitialConnection() {
    // Find the first controller that supports the extended gamepad profile
    if let controller = GCController.controllers().first(where: { $0.extendedGamepad != nil }) {
      connectToController(controller)
    } else {
      // No compatible controller found
      DispatchQueue.main.async {
        if self.isConnected { // Only update if state changes
          self.isConnected = false
          self.connectedControllerName = nil
          self.leftStickValues = nil
          self.rightStickValues = nil
          self.isButtonAPressed = false
          print("BluetoothStickController: No compatible controller found initially or after check.")
        }
      }
    }
  }
  
  /// Connects to a specific controller and sets up input handlers.
  private func connectToController(_ controller: GCController) {
    // If we are already connected to a different controller, disconnect from it first.
    if let currentCtrl = self.connectedController, currentCtrl != controller {
      print("BluetoothStickController: Switching controllers. Disconnecting from \(currentCtrl.vendorName ?? "old controller")")
      // This might trigger a disconnect notification which could call handleDisconnection.
      // For simplicity, we'll directly reset state here before connecting to the new one.
      self.handleDisconnection() // Reset state before connecting to new one.
    }
    
    guard let gamepad = controller.extendedGamepad else {
      print("BluetoothStickController: Controller \(controller.vendorName ?? "Unknown") does not support extended gamepad profile.")
      // If this was our connected controller, mark as disconnected
      if self.connectedController == controller {
        handleDisconnection()
      }
      return
    }
    
    self.connectedController = controller
    
    DispatchQueue.main.async {
      self.isConnected = true
      self.connectedControllerName = controller.vendorName
      print("BluetoothStickController: Successfully connected to \(controller.vendorName ?? "Unknown Vendor"). Setting up input handlers.")
      self.setupInputHandlers(for: gamepad)
      self.startControlUpdateTimer() // Start sending periodic updates
    }
  }
  
  // MARK: - Input Handling
  private func setupInputHandlers(for gamepad: GCExtendedGamepad) {
    // Initial state push
    DispatchQueue.main.async {
      let initialLeftStick = XY(
        x: Float32(self.applyDeadzone(gamepad.leftThumbstick.xAxis.value)),
        y: Float32(self.applyDeadzone(gamepad.leftThumbstick.yAxis.value))
      )
      let initialRightStick = XY(
        x: Float32(self.applyDeadzone(gamepad.rightThumbstick.xAxis.value)),
        y: Float32(self.applyDeadzone(gamepad.rightThumbstick.yAxis.value))
      )
      
      // Only update if different to avoid redundant SwiftUI updates
      if self.leftStickValues != initialLeftStick || self.rightStickValues != initialRightStick {
        self.leftStickValues = initialLeftStick
        self.rightStickValues = initialRightStick
      }
      
      if self.isButtonAPressed != gamepad.buttonA.isPressed {
        self.isButtonAPressed = gamepad.buttonA.isPressed
      }
    }
    
    // --- Left Thumbstick ---
    gamepad.leftThumbstick.valueChangedHandler = { [weak self] (dpad, xValue, yValue) in
      DispatchQueue.main.async {
        self?.handleStickValueChange(.left, x: xValue, y: yValue)
      }
    }
    
    // --- Right Thumbstick ---
    gamepad.rightThumbstick.valueChangedHandler = { [weak self] (dpad, xValue, yValue) in
      DispatchQueue.main.async {
        self?.handleStickValueChange(.right, x: xValue, y: yValue)
      }
    }
    
    // --- Button A ---
    gamepad.buttonA.pressedChangedHandler = { [weak self] (button, pressure, pressed) in
      DispatchQueue.main.async {
        self?.armButtonPressed(pressed)
      }
    }
  }
  
  private func applyDeadzone(_ value: Float) -> Float {
    if abs(value) < self.deadZone {
      return 0.0
    }
    return value
  }
  
  private func armButtonPressed(_ pressed: Bool) {
    if self.isButtonAPressed != pressed {
      self.isButtonAPressed = pressed
      if self.isButtonAPressed {
        armStatusListener?.handleArmStatusChange(sendUpdateToQuadcopter: true)
        print("Arm change")
      }
    }
  }
  
  private func handleStickValueChange(_ type : StickType, x : Float, y : Float)
  {
    let newX = self.applyDeadzone(x)
    let newY = self.applyDeadzone(y)
    let newStickValues = XY(x: Float32(newX), y: Float32(newY))
    
    switch type {
      case .left:
        if self.leftStickValues != newStickValues {
          self.leftStickValues = newStickValues
        }
        break
      case .right:
        if self.rightStickValues != newStickValues {
          self.rightStickValues = newStickValues
        }
        break
    }
    
    // No need to send updates here - the timer handles all transmissions at 10Hz
  }
}
