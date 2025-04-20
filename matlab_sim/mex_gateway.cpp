#include "mex.h"

#include <stdio.h>

#include "DebugHelper.h"
#include "QuadcopterController.h"
#include "Shared.h"
#include <string>

// Magnetometer
#include "Matrix.h"

#include <algorithm>

#include "Constants.h"
#include "ExtendedKalmanFilter.h"
#include "Filters/KalmanFilter.h"
#include "Filters/MedianFilter.h"
#include "Logger.h"

// TODO: These are defined in components/QMC5883L
// Refactor so that component respects MATLAB_SIM and
// can avoid duplicating these structs
struct xyz_vector_t {
  float x, y, z;
};

struct mag_update_t {
  float x, y, z;
  float heading;
};

static DebugHelper *_helper;
static QuadcopterController *_controller;

static void initController() {
  delete _controller;
  delete _helper;

  _helper = new DebugHelper();
  _controller = new QuadcopterController(_helper, micros());
}

static mag_update_t _magValues;
static imu_output_t _imuValues;
static EulerAngle _euler;
static ExtendedKalmanFilter::Config _ekfConfig;
static ExtendedKalmanFilter _extendedKalmanFilter(_ekfConfig);
static uint64_t _previousMicros = 0;
static bool _receivedImuUpdate = false;
static bool _gotFirstIMUUpdate = false;
static uint64_t _imuUpdateCounter = 0;
Vector3f _accelerometer;

static void _receivedIMUUpdate(imu_update_t update) {
  const float deltaTimeSeconds =
      (float)(micros() - _previousMicros) / 1000000.0f;
  _previousMicros = micros();

  Vector3f gyroscope = {update.gyro_x, update.gyro_y, update.gyro_z};
  _accelerometer = {update.accel_x, update.accel_y, update.accel_z};
  Vector3f mag = {_magValues.x, _magValues.y, _magValues.z};

  _extendedKalmanFilter.predict(
      gyroscope.x, gyroscope.y, gyroscope.z,
      _accelerometer.x * STANDARD_GRAVITY, _accelerometer.y * STANDARD_GRAVITY,
      _accelerometer.z * STANDARD_GRAVITY, deltaTimeSeconds);

  _extendedKalmanFilter.updateAccelerometer(
      _accelerometer.x * STANDARD_GRAVITY, _accelerometer.y * STANDARD_GRAVITY,
      _accelerometer.z * STANDARD_GRAVITY);

  const auto ekfAttitudeQuaternion =
      _extendedKalmanFilter.getAttitudeQuaternion();
  const auto ekfYawPitchRoll = _extendedKalmanFilter.getYawPitchRollDegrees();
  auto ekfAltitude = MAX(_extendedKalmanFilter.getAltitude(), 0.0f);
  auto ekfVerticalVelocity = _extendedKalmanFilter.getVerticalVelocity();

  _euler = {.yaw = ekfYawPitchRoll(0, 0),
            .pitch = ekfYawPitchRoll(1, 0),
            .roll = ekfYawPitchRoll(2, 0)};

  _imuValues = {
      .gyroOutput = {gyroscope.z, gyroscope.y, gyroscope.x},
      .yawPitchRollDegrees = {_euler.yaw, _euler.pitch, _euler.roll},
      .altitudeMeters = ekfAltitude,
      .verticalVelocityMetersPerSec = ekfVerticalVelocity,
  };

  _receivedImuUpdate = true;
  _gotFirstIMUUpdate = true;
}

// 0. timestamp
// 1. yaw
// 2. pitch
// 3. roll
// 4. raw accel x
// 5. raw accel y
// 6. raw accel z
// 7. raw gyro x
// 8. raw gyro y
// 9. raw gyro z
// 10. filtered accel x
// 11. filtered accel y
// 12. filtered accel z
// 13. filtered gyro x
// 14. filtered gyro y
// 15. filtered gyro z
// 16. angle out yaw
// 17. angle out pitch
// 18. angle out roll
// 19. rate out yaw
// 20. rate out pitch
// 21. rate out roll
// 22. motor 1
// 23. motor 2
// 24. motor 3
// 25. motor 4
// 26. mag x
// 27. mag y
// 28. mag z
// 29. mag heading
// 30. throttle
// 31. voltage
// 32. desired yaw
// 33. desired pitch
// 34. desired roll
// 35. ekf quat 0
// 36. ekf quat 1
// 37. ekf quat 2
// 38. ekf quat 3
// 39. ekf yaw
// 40. ekf pitch
// 41. ekf roll
// 42. ekf altitude
// 43. ekf vert velocity
// 44. relative altitude barometer (in meters)
// 45. relative altitude rangefinder (in meters)

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  initController();

  printf("Hello from C++! nlhs = %d, nrhs = %d\n", nlhs, nrhs);

  // Check the number of inputs
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("MyLibrary:processMatrix:InvalidNumInputs",
                      "One input required.");
  }

  // Ensure the input is a double matrix
  if (!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
    mexErrMsgIdAndTxt("MyLibrary:processMatrix:InvalidInputType",
                      "Input must be a real double matrix.");
  }

  // Get input matrix dimensions
  mwSize numRows = mxGetM(prhs[0]); // Number of rows in input matrix
  mwSize numCols = mxGetN(prhs[0]); // Number of columns in input matrix
  printf("numRows = %zu, numCols = %zu\n", numRows, numCols);

  // Get pointer to input data
  double *_data = mxGetPr(prhs[0]);

  // Print the first row for debugging
  printf("Data first row: ");
  for (mwSize col = 0; col < numCols; col++) {
    printf("%f ", _data[0 + col * numRows]); // Accessing element (0, col)
  }
  printf("\n");

  // Initialize output matrix with appropriate dimensions
  mwSize numOutputRows = numRows - 10; // Adjust based on your data processing
  mwSize numOutputCols = 10;
  plhs[0] = mxCreateDoubleMatrix(numOutputRows, numOutputCols, mxREAL);
  double *matrixData = mxGetPr(plhs[0]);

  printf("Processing data\n");
  for (mwSize row = 0; row < numOutputRows; row++) {
    // Set fake time using the first column (time data)
    double timeValue = _data[row + 0 * numRows];
    setFakeTime((uint64_t)timeValue);

    // Create imu_update_t struct with correct indexing
    imu_update_t update = {
        .accel_x = (float)_data[row + 4 * numRows],
        .accel_y = (float)_data[row + 5 * numRows],
        .accel_z = (float)_data[row + 6 * numRows],
        .gyro_x = (float)_data[row + 7 * numRows],
        .gyro_y = (float)_data[row + 8 * numRows],
        .gyro_z = (float)_data[row + 9 * numRows],
        .heading = (float)_data[row + 26 * numRows],
    };

    _extendedKalmanFilter.updateBarometer((float)_data[row + 44 * numRows]);
    _extendedKalmanFilter.updateRangefinder((float)_data[row + 45 * numRows]);

    if (row < 10) {
      printf("Update %zu: %f, %f, %f, %f, %f, %f\n", row + 1, update.accel_x,
             update.accel_y, update.accel_z, update.gyro_x, update.gyro_y,
             update.gyro_z);
    }

    // Process the IMU update
    _receivedIMUUpdate(update);

    // Write data to the output matrix using correct indexing
    matrixData[row + 0 * numOutputRows] = timeValue;                // Time
    matrixData[row + 1 * numOutputRows] = _imuValues.gyroOutput[0]; // Gyro X
    matrixData[row + 2 * numOutputRows] = _imuValues.gyroOutput[1]; // Gyro Y
    matrixData[row + 3 * numOutputRows] = _imuValues.gyroOutput[2]; // Gyro Z
    matrixData[row + 4 * numOutputRows] =
        _imuValues.yawPitchRollDegrees[0]; // Yaw
    matrixData[row + 5 * numOutputRows] =
        _imuValues.yawPitchRollDegrees[1]; // Pitch
    matrixData[row + 6 * numOutputRows] =
        _imuValues.yawPitchRollDegrees[2];                  // Roll
    matrixData[row + 7 * numOutputRows] = _accelerometer.x; // accel X
    matrixData[row + 8 * numOutputRows] = _accelerometer.y; // accel Y
    matrixData[row + 9 * numOutputRows] = _accelerometer.z; // accel Z
  }
}
