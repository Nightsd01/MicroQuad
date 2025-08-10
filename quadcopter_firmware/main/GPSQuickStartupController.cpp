#include "GPSQuickStartupController.h"

#ifndef MATLAB_SIM

#include <cstring>

#include "BLEController.h"
#include "BLEControllerLargeDataTransmissionHandler.h"
#include "BLELargeDataBlobType.h"
#include "GPSService.h"
#include "Logger.h"

GPSQuickStartupController::GPSQuickStartupController(BLEController* bleController, GPSService* gpsService)
    : _bleController(bleController), _gpsService(gpsService)
{
  _registerL2CAPListener();
}

GPSQuickStartupController::~GPSQuickStartupController() = default;

double GPSQuickStartupController::_readLittleEndianDouble(const uint8_t* bytes)
{
  uint64_t v = (static_cast<uint64_t>(bytes[0])) | (static_cast<uint64_t>(bytes[1]) << 8) |
               (static_cast<uint64_t>(bytes[2]) << 16) | (static_cast<uint64_t>(bytes[3]) << 24) |
               (static_cast<uint64_t>(bytes[4]) << 32) | (static_cast<uint64_t>(bytes[5]) << 40) |
               (static_cast<uint64_t>(bytes[6]) << 48) | (static_cast<uint64_t>(bytes[7]) << 56);
  double d = 0.0;
  std::memcpy(&d, &v, sizeof(double));
  return d;
}

void GPSQuickStartupController::_registerL2CAPListener()
{
  if (_bleController == nullptr) {
    LOG_ERROR("GPSQuickStartupController: BLEController is null; cannot register L2CAP listener");
    return;
  }
  BLEControllerLargeDataTransmissionHandler* l2cap = _bleController->getL2CAPHandler();
  if (l2cap == nullptr) {
    LOG_ERROR("GPSQuickStartupController: L2CAP handler not available");
    return;
  }

  l2cap->addLargeTransferReceivedListener([this](BLELargeDataBlobType type, const uint8_t* data, int64_t size) {
    if (type != BLELargeDataBlobType::GPSStartupFixData) {
      return;
    }
    if (data == nullptr || size < 24) {
      LOG_ERROR("GPSQuickStartupController: invalid GPS quick-start payload (size=%lld)", (long long)size);
      return;  // Need at least the 24-byte header
    }

    // Parse header
    const uint8_t* p = data;
    const double latitude = _readLittleEndianDouble(p);
    p += 8;
    const double longitude = _readLittleEndianDouble(p);
    p += 8;
    const double timestampSecs = _readLittleEndianDouble(p);
    p += 8;

    const size_t epoSize = static_cast<size_t>(size - 24);
    if (epoSize == 0) {
      LOG_ERROR("GPSQuickStartupController: EPO payload is empty");
      return;
    }

    // Copy ephemeris out of the NimBLE callback context to avoid blocking BLE task
    uint8_t* epoCopy = (uint8_t*)malloc(epoSize);
    if (!epoCopy) {
      LOG_ERROR("GPSQuickStartupController: failed to allocate %u bytes for EPO copy", (unsigned)epoSize);
      return;
    }
    memcpy(epoCopy, p, epoSize);

    LOG_INFO(
        "GPSQuickStartupController: received EPO blob (lat=%.6f, lon=%.6f, t=%.0f, bytes=%u) - deferring processing",
        latitude,
        longitude,
        timestampSecs,
        (unsigned)epoSize);

    // Defer heavy serial writes to the main scheduler to keep BLE responsive
    AsyncController::main.executeAfter(0, [this, epoCopy, epoSize, latitude, longitude, timestampSecs]() {
      if (_gpsService) {
        _gpsService->applyQuickStartupEphemeris(epoCopy, epoSize, latitude, longitude, timestampSecs);
        LOG_INFO("GPSQuickStartupController: quick-start data applied by GPSService");
      } else {
        LOG_ERROR("GPSQuickStartupController: GPSService is null; dropping EPO data");
      }
      free(epoCopy);
    });
  });
}

#endif  // MATLAB_SIM
