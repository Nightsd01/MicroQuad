#pragma once

#include <stdexcept>
#include <type_traits>
#include <utility>

#ifdef ESP_PLATFORM

#define USE_ACCELERATION 1

#include "esp_dsp.h"

template <typename T>
concept SupportedAccelerationTypes = std::same_as<T, float> || std::same_as<T, int16_t>;

template <SupportedAccelerationTypes T, size_t ROWSA, size_t INNERDIM, size_t COLSB>
inline void performMultiplication(const T *matrixA, const T *matrixB, T *result)
{
  esp_err_t error;
  if constexpr (std::is_same_v<T, float>) {
    error = dspm_mult_ex_f32(matrixA, matrixB, result, ROWSA, INNERDIM, COLSB, 0, 0, 0);
  } else {  // can only be int16_t here
    error = dspm_mult_s16(matrixA, matrixB, result, ROWSA, INNERDIM, COLSB, 0, 0, 0);
  }

  if (error != ESP_OK) {
    throw std::runtime_error("Matrix multiplication failed with error code: " + std::to_string(error));
  }
}

#endif  // ESP_PLATFORM