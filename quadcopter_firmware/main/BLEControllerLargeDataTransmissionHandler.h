#pragma once

#ifndef MATLAB_SIM

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

// NimBLE L2CAP includes
#include <host/ble_l2cap.h>

// Include for data blob types
#include "BLELargeDataBlobType.h"

// Forward declarations
class NimBLECharacteristic;

// Large transfer complete callback - called when a complete transfer is received
// Parameters: data type, data bytes (without type prefix), data size
using LargeTransferReceivedCallback = std::function<void(BLELargeDataBlobType type, const uint8_t* data, int64_t size)>;

class BLEControllerLargeDataTransmissionHandler
{
 public:
  BLEControllerLargeDataTransmissionHandler();
  BLEControllerLargeDataTransmissionHandler(uint16_t psm, uint16_t preferredMtu);
  ~BLEControllerLargeDataTransmissionHandler();

  // Initialize the L2CAP CoC server
  bool initialize();

  // Start listening for L2CAP connections
  bool startListening();

  // Stop listening and cleanup
  void stopListening();

  // Get the PSM value
  uint16_t getPSM() const { return _psm; }

  // Add listener for complete large data transfers
  void addLargeTransferReceivedListener(LargeTransferReceivedCallback callback)
  {
    _largeTransferReceivedCallback = callback;
  }

  // Transmission methods
  // Sends a large data blob. The first byte transmitted will be the provided
  // BLELargeDataBlobType so the receiver can identify the payload type.
  bool sendData(BLELargeDataBlobType type, const uint8_t* data, size_t length);
  bool isReadyToSend() const { return _channel != nullptr && !_txBusy; }

  // Get statistics
  size_t getBytesReceived() const { return _totalBytesReceived; }
  size_t getBytesSent() const { return _totalBytesSent; }
  size_t getExpectedBytes() const { return _expectedDataSize; }
  float getProgressPercentage() const
  {
    if (_expectedDataSize == 0) return 0.0f;
    return (float)_totalBytesReceived / (float)_expectedDataSize * 100.0f;
  }

  // Alternative: Process data chunks via BLE characteristic (fallback)
  bool processDataChunk(const uint8_t* data, size_t length);

  // Static callback for NimBLE L2CAP events
  static int l2capEventCallback(struct ble_l2cap_event* event, void* arg);

 private:
  // L2CAP channel info
  uint16_t _psm;  // Protocol/Service Multiplexer - must be in iOS dynamic range (0x0080-0x00FF)
  struct ble_l2cap_chan* _channel;
  uint16_t _mtu;

  uint32_t _expectedDataSize;
  size_t _totalBytesReceived;
  std::vector<uint8_t> _dataBuffer;

  // Header handling
  uint8_t _headerBuffer[5];
  size_t _headerBytesReceived;
  bool _headerComplete;
  BLELargeDataBlobType _incomingType;

  // Timing
  uint64_t _receptionStartTime;

  // Transmission state
  bool _txBusy;
  size_t _totalBytesSent;
  std::vector<uint8_t> _txQueue;  // Queue for pending data if channel is busy
  uint64_t _transmissionStartTime;

  // Callbacks
  LargeTransferReceivedCallback _largeTransferReceivedCallback;

  // Static instance for callbacks
  static BLEControllerLargeDataTransmissionHandler* _instance;

  // Helper methods
  void resetTransfer();
  void processHeader();
  void handleL2CAPData(struct os_mbuf* om);
  bool transmitFromQueue();

  // Alternative characteristic for fallback
  NimBLECharacteristic* _dataCharacteristic;

  // Friend class for access to private initialization
  friend class BLEController;
  bool initialize(NimBLECharacteristic* dataCharacteristic);
};

#endif  // MATLAB_SIM