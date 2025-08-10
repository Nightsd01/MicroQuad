#include "BLEControllerLargeDataTransmissionHandler.h"

#ifndef MATLAB_SIM

#include <Arduino.h>
#include <AsyncController.h>
#include <Logger.h>
#include <NimBLEDevice.h>
#include <host/ble_hs.h>
#include <os/os_mbuf.h>

#include <algorithm>
#include <cstring>

// Forward declaration of os_msys functions
extern "C"
{
  struct os_mbuf* os_msys_get_pkthdr(uint16_t dsize, uint16_t user_hdr_len);
  int ble_l2cap_send(struct ble_l2cap_chan* chan, struct os_mbuf* sdu_tx);
}

// Static instance pointer for callbacks
BLEControllerLargeDataTransmissionHandler* BLEControllerLargeDataTransmissionHandler::_instance = nullptr;

BLEControllerLargeDataTransmissionHandler::BLEControllerLargeDataTransmissionHandler()
    : _psm(0x0080),  // Dynamic PSM in iOS-allowed range (0x0080-0x00FF)
      _channel(nullptr),
      _mtu(498),  // Default L2CAP CoC MTU
      _expectedDataSize(0),
      _totalBytesReceived(0),
      _headerBytesReceived(0),
      _headerComplete(false),
      _receptionStartTime(0),
      _txBusy(false),
      _totalBytesSent(0),
      _transmissionStartTime(0),
      _dataCharacteristic(nullptr)
{
  _instance = this;
  LOG_INFO("L2CAP CoC handler initialized, will listen on PSM: 0x%04X", _psm);
}

BLEControllerLargeDataTransmissionHandler::BLEControllerLargeDataTransmissionHandler(
    uint16_t psm, uint16_t preferredMtu)
    : _psm(psm),
      _channel(nullptr),
      _mtu(preferredMtu),
      _expectedDataSize(0),
      _totalBytesReceived(0),
      _headerBytesReceived(0),
      _headerComplete(false),
      _receptionStartTime(0),
      _txBusy(false),
      _totalBytesSent(0),
      _transmissionStartTime(0),
      _dataCharacteristic(nullptr)
{
  _instance = this;
  LOG_INFO("L2CAP CoC handler initialized (custom), PSM: 0x%04X, preferred MTU: %u", _psm, _mtu);
}

BLEControllerLargeDataTransmissionHandler::~BLEControllerLargeDataTransmissionHandler()
{
  stopListening();
  if (_instance == this) {
    _instance = nullptr;
  }
}

bool BLEControllerLargeDataTransmissionHandler::initialize()
{
  // L2CAP CoC server will be created when a connection is established
  LOG_INFO("L2CAP CoC handler initialized, will listen on PSM: 0x%04X", _psm);
  return true;
}

bool BLEControllerLargeDataTransmissionHandler::initialize(NimBLECharacteristic* dataCharacteristic)
{
  _dataCharacteristic = dataCharacteristic;
  LOG_INFO("Large data handler initialized with characteristic fallback");
  return true;
}

bool BLEControllerLargeDataTransmissionHandler::startListening()
{
  // Check if we're connected
  NimBLEServer* server = NimBLEDevice::getServer();
  if (!server || server->getConnectedCount() == 0) {
    LOG_ERROR("Cannot start L2CAP server - no active connection");
    return false;
  }

  // Create L2CAP server
  int rc = ble_l2cap_create_server(_psm, _mtu, l2capEventCallback, this);
  if (rc != 0) {
    LOG_ERROR("Failed to create L2CAP server: %d", rc);
    return false;
  }

  LOG_INFO("L2CAP CoC server listening on PSM: 0x%04X", _psm);
  return true;
}

void BLEControllerLargeDataTransmissionHandler::stopListening()
{
  if (_channel) {
    ble_l2cap_disconnect(_channel);
    _channel = nullptr;
  }
  resetTransfer();
}

bool BLEControllerLargeDataTransmissionHandler::sendData(BLELargeDataBlobType type, const uint8_t* data, size_t length)
{
  if (!_channel) {
    LOG_ERROR("Cannot send data - L2CAP channel not connected");
    return false;
  }

  if (length == 0) {
    LOG_WARN("Attempted to send zero-length data");
    return false;
  }

  // Build transmission header: [type:1][payload_size:4 bytes LE]
  BLELargeDataBlobHeader header = {
      .type = type,
      .payloadSizeBytes = static_cast<uint32_t>(length),
  };

  // Queue header bytes
  const uint8_t* headerBytes = reinterpret_cast<const uint8_t*>(&header);
  _txQueue.insert(_txQueue.end(), headerBytes, headerBytes + sizeof(BLELargeDataBlobHeader));
  // Queue payload
  _txQueue.insert(_txQueue.end(), data, data + length);

  // Try to send immediately if not busy
  if (!_txBusy) {
    return transmitFromQueue();
  }

  // Data queued for later transmission
  return true;
}

bool BLEControllerLargeDataTransmissionHandler::transmitFromQueue()
{
  if (!_channel) {
    return true;
  }

  while (!_txQueue.empty()) {
    // Determine how much data to send (limited by MTU)
    const size_t bytesToSend = std::min(_txQueue.size(), (size_t)_mtu);

    // Allocate mbuf for transmission
    struct os_mbuf* om = os_msys_get_pkthdr(bytesToSend, 0);
    if (!om) {
      LOG_ERROR("Failed to allocate mbuf for L2CAP transmission");
      return false;
    }

    // Copy data to mbuf
    int rc = os_mbuf_append(om, _txQueue.data(), bytesToSend);
    if (rc != 0) {
      LOG_ERROR("Failed to append data to mbuf: %d", rc);
      os_mbuf_free_chain(om);
      return false;
    }

    // Send data over L2CAP channel
    rc = ble_l2cap_send(_channel, om);
    if (rc != 0) {
      if (rc == BLE_HS_ESTALLED) {
        // Channel is flow controlled, mark as busy and stop sending
        _txBusy = true;
        LOG_VERBOSE("L2CAP channel stalled, will resume on TX_UNSTALLED");
        return true;
      } else {
        LOG_ERROR("Failed to send L2CAP data: %d", rc);
        os_mbuf_free_chain(om);
        // Do not keep retrying immediately; leave queued, mark busy to wait for next opportunity
        _txBusy = true;
        return false;
      }
    }

    // Successfully queued data for transmission
    _txQueue.erase(_txQueue.begin(), _txQueue.begin() + bytesToSend);
    _totalBytesSent += bytesToSend;
    LOG_INFO("L2CAP TX: sent %zu bytes (MTU %u); remaining %zu", bytesToSend, _mtu, _txQueue.size());
  }

  return true;
}

void BLEControllerLargeDataTransmissionHandler::resetTransfer()
{
  _expectedDataSize = 0;
  _totalBytesReceived = 0;
  _headerBytesReceived = 0;
  _headerComplete = false;
  _dataBuffer.clear();
  memset(_headerBuffer, 0, sizeof(_headerBuffer));

  // Reset transmission state
  _txBusy = false;
  _totalBytesSent = 0;
  _txQueue.clear();
}

void BLEControllerLargeDataTransmissionHandler::processHeader()
{
  // Parse header: [type:1][size:4 LE]
  _incomingType = static_cast<BLELargeDataBlobType>(_headerBuffer[0]);
  _expectedDataSize = static_cast<uint32_t>(_headerBuffer[1]) | (static_cast<uint32_t>(_headerBuffer[2]) << 8) |
                      (static_cast<uint32_t>(_headerBuffer[3]) << 16) | (static_cast<uint32_t>(_headerBuffer[4]) << 24);

  LOG_INFO("Expecting %u bytes of data", _expectedDataSize);

  _dataBuffer.reserve(_expectedDataSize);
  _headerComplete = true;
  _receptionStartTime = millis();
}

void BLEControllerLargeDataTransmissionHandler::handleL2CAPData(struct os_mbuf* om)
{
  while (om) {
    uint8_t* data = (uint8_t*)om->om_data;
    uint16_t len = om->om_len;

    // Process header if not complete
    if (!_headerComplete) {
      size_t headerBytesNeeded = 5 - _headerBytesReceived;
      size_t headerBytesToCopy = (len < headerBytesNeeded) ? len : headerBytesNeeded;

      memcpy(&_headerBuffer[_headerBytesReceived], data, headerBytesToCopy);
      _headerBytesReceived += headerBytesToCopy;

      if (_headerBytesReceived == 5) {
        processHeader();
      }

      // Adjust data pointer and length for remaining data
      data += headerBytesToCopy;
      len -= headerBytesToCopy;
    }

    // Process data if header is complete and there's data remaining
    if (_headerComplete && len > 0) {
      _dataBuffer.insert(_dataBuffer.end(), data, data + len);
      _totalBytesReceived += len;

      // Log progress periodically
      static uint64_t lastProgressLog = 0;
      if (millis() - lastProgressLog > 1000) {
        float progress = getProgressPercentage();
        float throughput = 0;
        if (_receptionStartTime > 0) {
          float elapsedSeconds = (millis() - _receptionStartTime) / 1000.0f;
          if (elapsedSeconds > 0) {
            throughput = _totalBytesReceived / elapsedSeconds / 1024.0f;
          }
        }
        LOG_INFO(
            "L2CAP data reception: %.1f%% (%zu/%u bytes) @ %.1f KB/s",
            progress,
            _totalBytesReceived,
            _expectedDataSize,
            throughput);
        lastProgressLog = millis();
      }

      // Check if transfer is complete
      if (_totalBytesReceived >= _expectedDataSize) {
        // Call the large transfer callback if available (defer to avoid blocking BLE task)
        if (_largeTransferReceivedCallback) {
          auto type = _incomingType;
          // Copy buffer to heap to hand off safely
          std::vector<uint8_t> copy = _dataBuffer;  // shallow copy of vector storage
          AsyncController::main.executeAfter(0, [this, type, copy]() {
            if (_largeTransferReceivedCallback) {
              _largeTransferReceivedCallback(type, copy.data(), static_cast<int64_t>(copy.size()));
            }
          });
        }

        float totalTime = (millis() - _receptionStartTime) / 1000.0f;
        float avgThroughput = _totalBytesReceived / totalTime / 1024.0f;
        LOG_INFO(
            "L2CAP transfer complete: %zu bytes in %.2f seconds (%.1f KB/s)",
            _totalBytesReceived,
            totalTime,
            avgThroughput);

        // Reset for next transfer
        resetTransfer();
      }
    }

    om = SLIST_NEXT(om, om_next);
  }
}

// Static callback for NimBLE L2CAP events
int BLEControllerLargeDataTransmissionHandler::l2capEventCallback(struct ble_l2cap_event* event, void* arg)
{
  BLEControllerLargeDataTransmissionHandler* handler = static_cast<BLEControllerLargeDataTransmissionHandler*>(arg);

  switch (event->type) {
    case BLE_L2CAP_EVENT_COC_CONNECTED: {
      LOG_INFO("L2CAP CoC connected, channel=%p, handle=%d", event->connect.chan, event->connect.conn_handle);
      handler->_channel = event->connect.chan;
      handler->resetTransfer();
      // Get channel info including negotiated MTU
      struct ble_l2cap_chan_info info;
      ble_l2cap_get_chan_info(event->connect.chan, &info);

      // The effective MTU is the minimum of local and peer MTU
      uint16_t negotiated_mtu = (info.peer_coc_mtu < info.our_coc_mtu) ? info.peer_coc_mtu : info.our_coc_mtu;

      handler->_mtu = negotiated_mtu;

      LOG_INFO(
          "L2CAP MTU negotiated: Local=%u, Peer=%u, Effective=%u",
          info.our_coc_mtu,
          info.peer_coc_mtu,
          negotiated_mtu);

      // Provide an initial receive buffer
      // Allocate an mbuf sized for negotiated MTU to avoid repeated small reads
      struct os_mbuf* sdu_rx = os_msys_get_pkthdr(handler->_mtu, 0);
      if (sdu_rx) {
        ble_l2cap_recv_ready(event->connect.chan, sdu_rx);
      } else {
        LOG_ERROR("Failed to allocate initial L2CAP receive buffer for connected channel");
      }

      return 0;
    }

    case BLE_L2CAP_EVENT_COC_DISCONNECTED:
      LOG_INFO("L2CAP CoC disconnected");
      if (handler->_channel) {
        // Best-effort to drain/close cleanly
        handler->_channel = nullptr;
      }
      handler->resetTransfer();
      return 0;

    case BLE_L2CAP_EVENT_COC_DATA_RECEIVED: {
      // Handle incoming data
      handler->handleL2CAPData(event->receive.sdu_rx);

      // Free the mbuf
      os_mbuf_free_chain(event->receive.sdu_rx);

      // Provide a new receive buffer for the next data reception
      struct os_mbuf* sdu_rx = os_msys_get_pkthdr(handler->_mtu, 0);
      if (sdu_rx) {
        ble_l2cap_recv_ready(handler->_channel, sdu_rx);
      } else {
        LOG_ERROR("Failed to allocate new L2CAP receive buffer");
      }

      return 0;
    }

    case BLE_L2CAP_EVENT_COC_ACCEPT: {
      // Accept the incoming L2CAP connection
      LOG_INFO("L2CAP CoC accept event");

      // Accept the connection with our MTU
      event->accept.peer_sdu_size = handler->_mtu;
      // Also provision an initial receive buffer sized to MTU to avoid immediate starvation
      struct os_mbuf* sdu_rx = os_msys_get_pkthdr(handler->_mtu, 0);
      if (sdu_rx) {
        ble_l2cap_recv_ready(event->accept.chan, sdu_rx);
      }

      // Return 0 to accept the connection
      return 0;
    }

    case BLE_L2CAP_EVENT_COC_TX_UNSTALLED:
      // Channel is ready to send more data
      LOG_VERBOSE("L2CAP TX unstalled - resuming transmission");
      handler->_txBusy = false;

      // Resume transmission from queue
      handler->transmitFromQueue();
      return 0;

    default:
      LOG_ERROR("Unknown L2CAP event type: %d", event->type);
      return BLE_HS_EUNKNOWN;
  }
}

// Fallback: Process data chunks via BLE characteristic
bool BLEControllerLargeDataTransmissionHandler::processDataChunk(const uint8_t* data, size_t length)
{
  // This is the fallback method using BLE characteristics
  // It follows the same protocol as L2CAP CoC

  // Process header if not complete
  if (!_headerComplete) {
    size_t headerBytesNeeded = 4 - _headerBytesReceived;
    size_t headerBytesToCopy = (length < headerBytesNeeded) ? length : headerBytesNeeded;

    memcpy(&_headerBuffer[_headerBytesReceived], data, headerBytesToCopy);
    _headerBytesReceived += headerBytesToCopy;

    if (_headerBytesReceived == 4) {
      processHeader();
    }

    // Adjust data pointer and length for remaining data
    data += headerBytesToCopy;
    length -= headerBytesToCopy;
  }

  // Process data if header is complete and there's data remaining
  if (_headerComplete && length > 0) {
    _dataBuffer.insert(_dataBuffer.end(), data, data + length);
    _totalBytesReceived += length;

    // Check if transfer is complete
    if (_totalBytesReceived >= _expectedDataSize) {
      // Call the large transfer callback if available
      if (_largeTransferReceivedCallback && _dataBuffer.size() > 0) {
        // First byte is the data type
        BLELargeDataBlobType dataType = static_cast<BLELargeDataBlobType>(_dataBuffer[0]);

        // Call callback with type and data (excluding the type byte)
        if (_dataBuffer.size() > 1) {
          _largeTransferReceivedCallback(
              dataType,
              _dataBuffer.data() + 1,  // Skip the first byte (type)
              static_cast<int64_t>(_dataBuffer.size() - 1));
        }
      }

      LOG_INFO("BLE characteristic transfer complete: %zu bytes", _totalBytesReceived);

      // Reset for next transfer
      resetTransfer();

      // Send acknowledgment via characteristic
      if (_dataCharacteristic) {
        uint8_t ack = 0x01;  // Success
        _dataCharacteristic->setValue(&ack, 1);
        _dataCharacteristic->notify();
      }

      return true;
    }
  }

  // Transfer still in progress
  return false;
}

#endif  // MATLAB_SIM