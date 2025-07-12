#include "PersistentKeyValueStore.h"

#ifndef MATLAB_SIM

#include <nvs.h>
#include <nvs_flash.h>

#include <string>

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

std::string PersistentKeyValueStore::getStringForKey(const std::string& key)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in read mode
  err = nvs_open(kNamespaceName, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return "";
  }

  size_t required_size = 0;
  // Obtain required size first
  err = nvs_get_str(handle, key.c_str(), NULL, &required_size);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting string size!", esp_err_to_name(err));
    nvs_close(handle);
    return "";
  }

  char* value = new char[required_size];
  err = nvs_get_str(handle, key.c_str(), value, &required_size);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting string value!", esp_err_to_name(err));
    delete[] value;
    nvs_close(handle);
    return "";
  }

  std::string result(value);
  delete[] value;
  nvs_close(handle);
  return result;
}

int PersistentKeyValueStore::getIntForKey(const std::string& key)
{
  nvs_handle_t handle;
  esp_err_t err;
  int32_t value = 0;

  // Open NVS namespace in read mode
  err = nvs_open(kNamespaceName, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return 0;
  }

  err = nvs_get_i32(handle, key.c_str(), &value);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting int value!", esp_err_to_name(err));
    nvs_close(handle);
    return 0;
  }

  nvs_close(handle);
  return value;
}

float PersistentKeyValueStore::getFloatForKey(const std::string& key)
{
  nvs_handle_t handle;
  esp_err_t err;
  float value = 0.0f;

  // Open NVS namespace in read mode
  err = nvs_open(kNamespaceName, NVS_READONLY, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return 0.0f;
  }

  // NVS doesn't support float directly, store as string
  size_t required_size = 0;
  err = nvs_get_str(handle, key.c_str(), NULL, &required_size);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting float size!", esp_err_to_name(err));
    nvs_close(handle);
    return 0.0f;
  }

  char* str_value = new char[required_size];
  err = nvs_get_str(handle, key.c_str(), str_value, &required_size);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) getting float value!", esp_err_to_name(err));
    delete[] str_value;
    nvs_close(handle);
    return 0.0f;
  }

  value = std::stof(str_value);
  delete[] str_value;
  nvs_close(handle);
  return value;
}

void PersistentKeyValueStore::setStringForKey(const std::string& key, const std::string& value)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  err = nvs_set_str(handle, key.c_str(), value.c_str());
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) setting string value!", esp_err_to_name(err));
  }

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) committing changes!", esp_err_to_name(err));
  }

  nvs_close(handle);
}

void PersistentKeyValueStore::setIntForKey(const std::string& key, int value)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  err = nvs_set_i32(handle, key.c_str(), value);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) setting int value!", esp_err_to_name(err));
  }

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) committing changes!", esp_err_to_name(err));
  }

  nvs_close(handle);
}

void PersistentKeyValueStore::setFloatForKey(const std::string& key, float value)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Convert float to string for storage
  std::string str_value = std::to_string(value);

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  err = nvs_set_str(handle, key.c_str(), str_value.c_str());
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) setting float value!", esp_err_to_name(err));
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

template <typename T>
void PersistentKeyValueStore::setVectorForKey(const std::string& key, const std::vector<T>& value)
{
  nvs_handle_t handle;
  esp_err_t err;

  // Open NVS namespace in write mode
  err = nvs_open(kNamespaceName, NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) opening NVS handle!", esp_err_to_name(err));
    return;
  }

  err = nvs_set_blob(handle, key.c_str(), value.data(), value.size() * sizeof(T));
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) setting vector value!", esp_err_to_name(err));
  }

  // Commit changes
  err = nvs_commit(handle);
  if (err != ESP_OK) {
    LOG_ERROR("Error (%s) committing changes!", esp_err_to_name(err));
  }

  LOG_INFO("Stored vector with values (for key %s): ", key.c_str());
  nvs_close(handle);
}

template <typename T>
std::vector<T> PersistentKeyValueStore::getVectorForKey(const std::string& key, size_t length)
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
      LOG_INFO("%f ", result[i]);
    }
  }

  // If we reach here, err was ESP_OK and the size matches.
  return result;
}

template std::vector<float> PersistentKeyValueStore::getVectorForKey(const std::string& key, size_t length);
template void PersistentKeyValueStore::setVectorForKey(const std::string& key, const std::vector<float>& value);

template std::vector<double> PersistentKeyValueStore::getVectorForKey(const std::string& key, size_t length);
template void PersistentKeyValueStore::setVectorForKey(const std::string& key, const std::vector<double>& value);

#endif  // MATLAB_SIM