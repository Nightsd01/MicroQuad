//
//  CalibrationValues.swift
//  QuadController
//
//  Created by Brad Hesse on 11/2/24.
//

import Foundation

struct CalibrationValues {
  let gyroBiasX, gyroBiasY, gyroBiasZ : Float
  let accelBiasX, accelBiasY, accelBiasZ : Float
  let accelScaleX, accelScaleY, accelScaleZ : Float
  let gyroOffsetX, gyroOffsetY, gyroOffsetZ : Int16
  let accelOffsetX, accelOffsetY, accelOffsetZ : Int16
  
  init(_ data : Data) {
    // NOTE: Keep this aligned with calibration_data_t in IMU.h
    //    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
    //    float accel_bias_x, accel_bias_y, accel_bias_z;
    //    float accel_scale_x, accel_scale_y, accel_scale_z;
    //    int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
    //    int16_t accel_raw_x, accel_raw_y, accel_raw_z;
    //    bool success;
    
    let floats: [Float32] = data.prefix(9 * 4).withUnsafeBytes { bytes in
        Array(UnsafeBufferPointer(
            start: bytes.baseAddress?.assumingMemoryBound(to: Float32.self),
            count: 9
        ))
    }
    
    // Get just the Int16s (6 int16s × 2 bytes each = 12 bytes)
    let int16Offset = 9 * 4  // Start after the floats
    let int16s: [Int16] = data.suffix(from: int16Offset).prefix(6 * 2).withUnsafeBytes { bytes in
        Array(UnsafeBufferPointer(
            start: bytes.baseAddress?.assumingMemoryBound(to: Int16.self),
            count: 6
        ))
    }
    
    gyroBiasX = floats[0]
    gyroBiasY = floats[1]
    gyroBiasZ = floats[2]
    accelBiasX = floats[3]
    accelBiasY = floats[4]
    accelBiasZ = floats[5]
    accelScaleX = floats[6]
    accelScaleY = floats[7]
    accelScaleZ = floats[8]
    gyroOffsetX = int16s[0]
    gyroOffsetY = int16s[1]
    gyroOffsetZ = int16s[2]
    accelOffsetX = int16s[3]
    accelOffsetY = int16s[4]
    accelOffsetZ = int16s[5]
  }
}
