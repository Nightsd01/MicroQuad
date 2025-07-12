#include "PersistentKeyValueStore.h"

#ifndef MATLAB_SIM

#include <nvs.h>
#include <nvs_flash.h>

#include <string>
#include <type_traits>

#include "Logger.h"

static const char* kNamespaceName = "kv_storage";

PersistentKeyValueStore::PersistentKeyValueStore()
{
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated
    // and needs to be erased
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

// Generic getValue implementation
template <typename T>
T PersistentKeyValueStore::getValue(const std::string& key)
{
  nvs_handle_t handle;
  esp_err_t err;
  T value = T{};

  // Open NVS namespace in read mode
  err = nvs_open(kNamespaceName, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return T{};
  }

  if constexpr (std::is_same_v<T, std::string>) {
    // Handle string type
    size_t required_size = 0;
    err = nvs_get_str(handle, key.c_str(), NULL, &required_size);
    if (err == ESP_OK && required_size > 0) {
      char* str_value = new char[required_size];
      err = nvs_get_str(handle, key.c_str(), str_value, &required_size);
      if (err == ESP_OK) {
        value = std::string(str_value);
      }
      delete[] str_value;
    }
  } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    // Handle integer types
    if constexpr (sizeof(T) <= 4) {
      // 32-bit or smaller integers
      int32_t temp_value = 0;
      err = nvs_get_i32(handle, key.c_str(), &temp_value);
      if (err == ESP_OK) {
        value = static_cast<T>(temp_value);
      }
    } else {
      // 64-bit integers - store as blob
      size_t length = sizeof(T);
      err = nvs_get_blob(handle, key.c_str(), &value, &length);
      if (err != ESP_OK || length != sizeof(T)) {
        err = ESP_FAIL;
      }
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    // Handle floating point types - store as string for precision
    size_t required_size = 0;
    err = nvs_get_str(handle, key.c_str(), NULL, &required_size);
    if (err == ESP_OK && required_size > 0) {
      char* str_value = new char[required_size];
      err = nvs_get_str(handle, key.c_str(), str_value, &required_size);
      if (err == ESP_OK) {
        if constexpr (std::is_same_v<T, float>) {
          value = std::stof(str_value);
        } else if constexpr (std::is_same_v<T, double>) {
          value = std::stod(str_value);
        } else if constexpr (std::is_same_v<T, long double>) {
          value = std::stold(str_value);
        }
      }
      delete[] str_value;
    }
  } else {
    // For other types, try to load as blob
    size_t length = sizeof(T);
    err = nvs_get_blob(handle, key.c_str(), &value, &length);
    if (err != ESP_OK || length != sizeof(T)) {
      err = ESP_FAIL;
    }
  }

  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting value for key '%s'!", esp_err_to_name(err), key.c_str());
  }

  nvs_close(handle);
  return value;
}

// Special overload for vectors
template <typename T>
std::vector<T> PersistentKeyValueStore::getValue(const std::string& key, size_t length)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in read mode
  err = nvs_open(kNamespaceName, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle for key '%s'!", esp_err_to_name(err), key.c_str());
    return {};  // Return empty vector on error
  }

  // Calculate the expected size in bytes
  size_t expectedSizeBytes = length * sizeof(T);
  if (expectedSizeBytes == 0 && length > 0) {
    // Avoid potential issues if sizeof(T) is somehow zero or length is huge causing overflow
    LOG_ERROR("Invalid expected size calculation for key '%s'", key.c_str());
    nvs_close(handle);
    return {};
  }

  // Create the vector to hold the result. It's initialized with default values.
  std::vector<T> result(length);

  // Prepare the size variable for nvs_get_blob (in/out parameter)
  // Input: size of the buffer (result.data())
  // Output: size of the blob actually read
  size_t foundSizeBytes = expectedSizeBytes;  // Initialize with buffer size

  // Attempt to read the blob directly into the vector's memory
  // Only proceed if the vector has allocated space (length > 0)
  if (length > 0) {
    err = nvs_get_blob(handle, key.c_str(), result.data(), &foundSizeBytes);
  } else {
    // If expected length is 0, check if a zero-length blob exists
    err = nvs_get_blob(handle, key.c_str(), nullptr, &foundSizeBytes);
    if (err == ESP_OK && foundSizeBytes == 0) {
      // Key exists and is zero-length, which matches expectation
      // result is already empty, so this is fine.
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
      // Key doesn't exist, also fine if length is 0.
      err = ESP_OK;  // Treat as success
    } else {
      // Some other error occurred or non-zero blob found when expecting zero
      err = ESP_FAIL;  // Force error path
    }
  }

  nvs_close(handle);  // Close NVS handle regardless of success or failure

  // --- Check results ---
  if (err != ESP_OK) {
    // Common case: Key not found - don't log as error unless debugging
    if (err == ESP_ERR_NVS_NOT_FOUND) {
      // LOG_DEBUG("Key '%s' not found in NVS.", key.c_str());
    } else {
      LOG_ERROR("Error (%s) getting blob for key '%s'!", esp_err_to_name(err), key.c_str());
    }
    return {};  // Return empty vector on error
  }

  // *** Crucial Check: Verify the size actually read ***
  if (foundSizeBytes != expectedSizeBytes) {
    LOG_ERROR(
        "Size mismatch for key '%s': Expected %zu bytes, but found %zu bytes!",
        key.c_str(),
        expectedSizeBytes,
        foundSizeBytes);
    return {};  // Return empty vector because data is incomplete/corrupt
  } else {
    LOG_INFO("Loaded vector with values (for key %s): ", key.c_str());
    for (size_t i = 0; i < length; ++i) {
      if constexpr (std::is_floating_point_v<T>) {
        LOG_INFO("%f ", static_cast<double>(result[i]));
      } else {
        LOG_INFO("%lld ", static_cast<long long>(result[i]));
      }
    }
  }

  // If we reach here, err was ESP_OK and the size matches.
  return result;
}

// Generic setValue implementation
template <typename T>
void PersistentKeyValueStore::setValue(const std::string& key, const T& value)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    // Handle string type
    err = nvs_set_str(handle, key.c_str(), value.c_str());
  } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
    // Handle integer types
    if constexpr (sizeof(T) <= 4) {
      // 32-bit or smaller integers
      err = nvs_set_i32(handle, key.c_str(), static_cast<int32_t>(value));
    } else {
      // 64-bit integers - store as blob
      err = nvs_set_blob(handle, key.c_str(), &value, sizeof(T));
    }
  } else if constexpr (std::is_floating_point_v<T>) {
    // Handle floating point types - store as string for precision
    std::string str_value = std::to_string(value);
    err = nvs_set_str(handle, key.c_str(), str_value.c_str());
  } else if constexpr (requires {
                         value.data();
                         value.size();
                       }) {
    // Handle vector-like types (has data() and size() methods)
    err = nvs_set_blob(handle, key.c_str(), value.data(), value.size() * sizeof(typename T::value_type));
    if (err == ESP_OK) {
      LOG_INFO("Stored vector with %zu elements (for key %s)", value.size(), key.c_str());
    }
  } else {
    // For other types, try to store as blob
    err = nvs_set_blob(handle, key.c_str(), &value, sizeof(T));
  }

  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) setting value for key '%s'!", esp_err_to_name(err), key.c_str());
  }

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) committing changes!", esp_err_to_name(err));
  }

  nvs_close(handle);
}

bool PersistentKeyValueStore::hasValueForKey(const std::string& key)
{
  nvs_iterator_t it = nullptr;
  esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, kNamespaceName, NVS_TYPE_ANY, &it);
  if (err != ESP_OK || it == NULL) {
    // No entries found in the namespace or an error occurred
    return false;
  }

  bool found = false;
  while (it != NULL) {
    nvs_entry_info_t info;
    nvs_entry_info(it, &info);
    if (key == info.key) {
      found = true;
      nvs_release_iterator(it);  // Release the iterator if we found the key
      break;
    }
    err = nvs_entry_next(&it);  // Advances and releases the current iterator
    if (err != ESP_OK || it == NULL) {
      // No entries found in the namespace or an error occurred
      return false;
    }
  }

  // If we didn't find the key, the last iterator is already released
  return found;
}

void PersistentKeyValueStore::removeValueForKey(const std::string& key)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  err = nvs_erase_key(handle, key.c_str());
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) erasing key!", esp_err_to_name(err));
  }

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) committing erase!", esp_err_to_name(err));
  }

  nvs_close(handle);
}

// Explicit template instantiations for getValue/setValue
// Arithmetic types
template int PersistentKeyValueStore::getValue<int>(const std::string& key);
template void PersistentKeyValueStore::setValue<int>(const std::string& key, const int& value);

template int8_t PersistentKeyValueStore::getValue<int8_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<int8_t>(const std::string& key, const int8_t& value);

template int16_t PersistentKeyValueStore::getValue<int16_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<int16_t>(const std::string& key, const int16_t& value);

// Note: On many platforms, int and int32_t are the same type, so we skip int32_t

template int64_t PersistentKeyValueStore::getValue<int64_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<int64_t>(const std::string& key, const int64_t& value);

template uint8_t PersistentKeyValueStore::getValue<uint8_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<uint8_t>(const std::string& key, const uint8_t& value);

template uint16_t PersistentKeyValueStore::getValue<uint16_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<uint16_t>(const std::string& key, const uint16_t& value);

template uint32_t PersistentKeyValueStore::getValue<uint32_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<uint32_t>(const std::string& key, const uint32_t& value);

template uint64_t PersistentKeyValueStore::getValue<uint64_t>(const std::string& key);
template void PersistentKeyValueStore::setValue<uint64_t>(const std::string& key, const uint64_t& value);

template float PersistentKeyValueStore::getValue<float>(const std::string& key);
template void PersistentKeyValueStore::setValue<float>(const std::string& key, const float& value);

template double PersistentKeyValueStore::getValue<double>(const std::string& key);
template void PersistentKeyValueStore::setValue<double>(const std::string& key, const double& value);

// String type
template std::string PersistentKeyValueStore::getValue<std::string>(const std::string& key);
template void PersistentKeyValueStore::setValue<std::string>(const std::string& key, const std::string& value);

// Vector types with getValue(key, length)
template std::vector<float> PersistentKeyValueStore::getValue<float>(const std::string& key, size_t length);
template std::vector<double> PersistentKeyValueStore::getValue<double>(const std::string& key, size_t length);
template std::vector<int> PersistentKeyValueStore::getValue<int>(const std::string& key, size_t length);

// Vector types with setValue
template void PersistentKeyValueStore::setValue<std::vector<float>>(
    const std::string& key, const std::vector<float>& value);
template void PersistentKeyValueStore::setValue<std::vector<double>>(
    const std::string& key, const std::vector<double>& value);
template void PersistentKeyValueStore::setValue<std::vector<int>>(
    const std::string& key, const std::vector<int>& value);

#endif  // MATLAB_SIM