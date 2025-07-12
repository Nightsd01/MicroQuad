#include "MotorMagCompensationHandler.h"

#include "PersistentKeysCommon.h"

#ifndef MATLAB_SIM

// --- Static Helper Functions ---

float MotorMagCompensationHandler::mapRange(float value, float inMin, float inMax, float outMin, float outMax)
{
  // Clamp value to input range to prevent extrapolation issues
  if (value < inMin) value = inMin;
  if (value > inMax) value = inMax;

  // Prevent division by zero if input range is zero
  if (std::abs(inMax - inMin) < 1e-6f) {
    return outMin;  // Or return average (outMin + outMax) / 2.0f
  }

  return outMin + (outMax - outMin) * ((value - inMin) / (inMax - inMin));
}

mag_update_t MotorMagCompensationHandler::lerpMag(const mag_update_t &a, const mag_update_t &b, float t)  // 0 <= t <= 1
{
  // Clamp t to [0, 1] to ensure interpolation stays within bounds
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  mag_update_t m;
  m.x = a.x + t * (b.x - a.x);
  m.y = a.y + t * (b.y - a.y);
  m.z = a.z + t * (b.z - a.z);
  m.heading = 0;  // Heading is recalculated after compensation
  return m;
}

float MotorMagCompensationHandler::_headingFromXY(float x, float y)
{
  // Assumes DECLINATION_ANGLE_DEG, DEG_TO_RADS, PI are defined correctly
  constexpr float declinationRad = DEG_TO_RADS(DECLINATION_ANGLE_DEG);

  // Calculate heading using atan2, adjust based on magnetometer orientation relative to NED frame.
  // This assumes Magnetometer Y points East, X points North for standard NED atan2(East, North) -> atan2(y, x).
  // The original code used atan2(-y, x). Verify which axis points where on your hardware.
  // Using atan2(y, x) for standard mathematical/NED convention:
  float hdg_rad = atan2(y, x);

  hdg_rad += declinationRad;

  // Normalize heading to [0, 2*PI) radians
  hdg_rad = fmod(hdg_rad, 2.0f * PI);
  if (hdg_rad < 0) {
    hdg_rad += 2.0f * PI;
  }

  // Convert to degrees [0, 360)
  return hdg_rad * 180.0f / PI;
}

// --- Class Methods ---

MotorMagCompensationHandler::MotorMagCompensationHandler(PersistentKeyValueStore *kvStore)
    :                                 // Initializer list MUST match declaration order in MotorMagCompensationHandler.h
      isCalibrating(false),           // 1st declared member (public)
      isCalibrated(false),            // 2nd declared member (public, assumed position)
      _kvStore(kvStore),              // Next declared member (private)
      _motorOutputCallback(nullptr),  // Initialize function pointers safely
      _calibrationCompleteCallback(nullptr),  // Initialize function pointers safely
      _baselineMag(),                         // Default initialize
      _baselineCaptured(false),               // Initialize flag
      _lastStepTs(0),                         // Initialize primitive types
      _curStep(0),
      _stepAcc(),  // Default initialize struct
      _spinUpStartTs(0),
      _inSpinUp(false),
      _calibrationData()  // Default initialize array (zeroes it out)
{
  // Load calibration data (X, Y, Z offsets) for each step
  for (int step = 0; step < CALIBRATION_MOTOR_STEPS; ++step) {
    std::string key = _getKey(step);
    if (!_kvStore->hasValueForKey(key)) {
      // Data missing for this step, calibration is invalid
      isCalibrated = false;
      return;  // Stop loading
    }

    // Expecting vector of [x_offset, y_offset, z_offset]
    std::vector<float> xyzOffsets = _kvStore->getValue<float>(key, 3 /* expected length */);
    if (xyzOffsets.size() != 3) {
      // Data corrupted or wrong format
      isCalibrated = false;
      return;  // Stop loading
    }

    // Store the loaded offsets
    _calibrationData[step] = {
        .x = xyzOffsets[0],
        .y = xyzOffsets[1],
        .z = xyzOffsets[2],
        .heading = 0.0f  // Placeholder, calculated dynamically if needed from meanMag
    };
  }

  // If loop completed without returning, all data was loaded successfully
  isCalibrated = true;
}

std::string MotorMagCompensationHandler::_getKey(int step)
{
  // Assumes PersistentKeysCommon::MAG_COMPENSATION_OFFSET_PREFIX is defined
  return PersistentKeysCommon::MAG_COMPENSATION_OFFSET_PREFIX + "s" + std::to_string(step);
}

void MotorMagCompensationHandler::beginCalibration(
    std::function<void(motor_outputs_t)> motorOutputCallback, std::function<void(bool)> calibrationCompleteCallback)
{
  isCalibrating = true;
  isCalibrated = false;
  _baselineCaptured = false;  // Reset baseline flag
  _motorOutputCallback = motorOutputCallback;
  _calibrationCompleteCallback = calibrationCompleteCallback;
  _curStep = 0;  // Start at step 0 for baseline
  _stepAcc.reset();
  _inSpinUp = false;  // No spin-up for baseline measurement

  // Set all motors to ZERO for baseline measurement
  motor_outputs_t zeroThrottles{};
  std::fill(std::begin(zeroThrottles), std::end(zeroThrottles), 0.0f);  // Use 0.0f for baseline
  _motorOutputCallback(zeroThrottles);

  _lastStepTs = millis();  // Start measurement window immediately for baseline
}

void MotorMagCompensationHandler::updateMagValue(const mag_update_t &mag)
{
  if (!isCalibrating) return;

  const uint32_t now = millis();

  // --- 0. Handle Spin-up Period (Skip for baseline step 0) ---
  if (_curStep > 0 && _inSpinUp) {  // Only apply spin-up delay after baseline
    if (now - _spinUpStartTs < SPINUP_MS) {
      return;  // Still spinning up, ignore sample
    }
    // Spin-up finished for this step
    _inSpinUp = false;
    _lastStepTs = now;  // Start the actual measurement window now
    _stepAcc.reset();   // Reset accumulator for the measurement window
  } else if (_curStep == 0 && _inSpinUp) {
    // Should not happen if beginCalibration sets _inSpinUp = false initially
    // but as safety:
    _inSpinUp = false;
    _lastStepTs = now;
    _stepAcc.reset();
  }

  // --- 1. Accumulate Sample ---
  // Skip accumulation if we are in spin-up for steps > 0
  if (!(_curStep > 0 && _inSpinUp)) {
    _stepAcc.accumulate(mag);
  }

  // --- 2. Check if Step Time Window is Complete ---
  if (now - _lastStepTs < CALIBRATION_STEP_TIME_MS) {
    return;  // Still collecting data for this step
  }

  // --- 3. Step Time Window Finished: Store Results ---
  if (_stepAcc.n > 0) {
    mag_update_t meanMag = _stepAcc.meanMag();

    if (_curStep == 0) {
      // This was the baseline measurement (motors off)
      _baselineMag = meanMag;
      _baselineCaptured = true;
      // Don't store baseline in the _calibrationData array itself
      // _calibrationData will store *offsets* relative to baseline.
      // Initialize first offset entry (corresponding to THROTTLE_MIN) perhaps?
      // Or adjust indexing later. Let's store offsets starting from index 0,
      // corresponding to the first *non-zero* throttle step.
    } else {
      // This is a motor-on step. Calculate and store the offset.
      // Check if baseline was captured before proceeding.
      if (!_baselineCaptured) {
        // Error: Trying to calculate offset without a baseline. Abort.
        isCalibrating = false;
        if (_calibrationCompleteCallback) _calibrationCompleteCallback(false);  // Signal failure
        // Consider logging an error here
        return;
      }

      // Calculate the offset relative to baseline
      mag_update_t offsetMag = {
          .x = meanMag.x - _baselineMag.x,
          .y = meanMag.y - _baselineMag.y,
          .z = meanMag.z - _baselineMag.z,
          .heading = 0.0f  // Heading not relevant for offset vector
      };
      // Store offset. Adjust index because step 0 was baseline.
      // Step 1 (first motor step) goes into index 0.
      _calibrationData[_curStep - 1] = offsetMag;
    }
  } else {
    // Handle no samples collected (as before, or signal error)
    // If this happens for baseline (_curStep == 0), calibration cannot proceed.
    if (_curStep == 0) {
      isCalibrating = false;
      if (_calibrationCompleteCallback) _calibrationCompleteCallback(false);  // Signal failure
      // Log error
      return;
    } else {
      // Store zero offset if no samples for a motor step? Risky. Maybe abort.
      _calibrationData[_curStep - 1] = {0.0f, 0.0f, 0.0f, 0.0f};
    }
  }

  // --- 4. Advance to Next Step or Complete Calibration ---
  _stepAcc.reset();  // Reset accumulator for the next step/spin-up

  ++_curStep;
  // Need CALIBRATION_MOTOR_STEPS offsets, corresponding to steps 1 to CALIBRATION_MOTOR_STEPS.
  // So we stop when _curStep goes *beyond* CALIBRATION_MOTOR_STEPS.
  if (_curStep > CALIBRATION_MOTOR_STEPS) {  // Changed >= to > because we have steps 1..N now.
    // All steps completed (Baseline + N motor steps)
    _completeCalibration();
    return;
  }

  // --- 5. Set Motors for Next Step and Start Spin-up ---
  // Calculate throttle for the *actual* motor step number (1 to N)
  // Map range from step 1 to CALIBRATION_MOTOR_STEPS
  const float step_for_throttle = static_cast<float>(_curStep);  // Current step number (1, 2, ...)
  const float total_motor_steps = static_cast<float>(CALIBRATION_MOTOR_STEPS);

  // Map step number (1..N) to throttle range [THROTTLE_MIN, THROTTLE_MAX]
  const float nextDuty = mapRange(
      step_for_throttle,  // Input value (1 to N)
      1.0f,               // Input min
      total_motor_steps,  // Input max
      THROTTLE_MIN,       // Output min
      THROTTLE_MAX);      // Output max

  motor_outputs_t throttles{};
  std::fill(std::begin(throttles), std::end(throttles), nextDuty);
  _motorOutputCallback(throttles);

  // Start spin-up for the new step (only for motor steps, i.e., _curStep > 0)
  _inSpinUp = true;
  _spinUpStartTs = now;  // Use current time as start of spin-up
}

mag_update_t MotorMagCompensationHandler::applyMagneticMotorCompensation(
    const mag_update_t &rawMag, const motor_outputs_t &motorThrottles)
{
  if (!isCalibrated) {
    // Not calibrated, return raw data with calculated heading
    mag_update_t uncompensatedMag = rawMag;
    uncompensatedMag.heading = _headingFromXY(rawMag.x, rawMag.y);
    return uncompensatedMag;
  }

  // --- 1. Calculate Average Throttle ---
  float throttleSum = std::accumulate(motorThrottles.begin(), motorThrottles.end(), 0.0f);
  // Assuming NUM_MOTORS is defined correctly (e.g., 4)
  float avgThrottle = (NUM_MOTORS > 0) ? (throttleSum / static_cast<float>(NUM_MOTORS)) : 0.0f;

  // --- 2. Normalize Average Throttle ---
  // Map avgThrottle from [THROTTLE_MIN, THROTTLE_MAX] to index range [0, N-1]
  // Note: The calibration data indices 0..N-1 correspond to throttle steps
  // that were originally mapped from step numbers 1..N.
  const float normalized_throttle_for_index = mapRange(
      avgThrottle,
      THROTTLE_MIN,
      THROTTLE_MAX,  // The actual throttle range measured
      0.0f,
      static_cast<float>(CALIBRATION_MOTOR_STEPS - 1)  // Map to the array index range [0, N-1]
  );

  // --- 3. Find Surrounding Calibration Steps ---
  const float idx = normalized_throttle_for_index;  // Use the correctly mapped index
  const int k0 = static_cast<int>(std::floor(idx));
  // Ensure k0 is within bounds [0, N-1]
  const int k0_clamped = std::max(0, std::min(k0, CALIBRATION_MOTOR_STEPS - 1));
  // Ensure k1 is within bounds [0, N-1]
  const int k1_clamped = std::min(k0_clamped + 1, CALIBRATION_MOTOR_STEPS - 1);
  const float alpha = idx - k0;  // Interpolation factor [0, 1] (clamp if needed)
  const float alpha_clamped = std::max(0.0f, std::min(alpha, 1.0f));

  // --- 4. Interpolate Offset from Calibration Data ---
  // These are now OFFSET vectors
  const auto &offset0 = _calibrationData[k0_clamped];
  const auto &offset1 = _calibrationData[k1_clamped];
  // Interpolated *motor offset*
  const mag_update_t estimatedMotorOffset = lerpMag(offset0, offset1, alpha_clamped);

  // --- 5. Subtract Interpolated Offset from Raw Reading ---
  mag_update_t compensatedMag = {
      .x = rawMag.x - estimatedMotorOffset.x,
      .y = rawMag.y - estimatedMotorOffset.y,
      .z = rawMag.z - estimatedMotorOffset.z,
      .heading = 0  // Will calculate next
  };

  // --- 6. Recalculate Heading ---
  // This uses the corrected ambient field components
  compensatedMag.heading = _headingFromXY(compensatedMag.x, compensatedMag.y);

  return compensatedMag;
}

void MotorMagCompensationHandler::_completeCalibration()
{
  isCalibrating = false;  // Calibration process finished

  bool saveSuccess = true;
  // Save the collected calibration data (magnetic field offsets)
  for (int step = 0; step < CALIBRATION_MOTOR_STEPS; ++step) {
    const auto &bias = _calibrationData[step];

    // Store only X, Y, Z components of the bias vector
    std::vector<float> record = {bias.x, bias.y, bias.z};

    std::string key = _getKey(step);
    // Store the compensation values
    _kvStore->setValue(key, record);
  }

  // Update calibration status based on whether saving was successful
  isCalibrated = saveSuccess;

  // Notify the caller about completion status
  if (_calibrationCompleteCallback) {
    _calibrationCompleteCallback(saveSuccess);
  }

  // Reset state variables for potential next calibration run
  _curStep = 0;
  _inSpinUp = false;
  _stepAcc.reset();
  _lastStepTs = 0;
  _spinUpStartTs = 0;
}

#endif  // MATLAB_SIM