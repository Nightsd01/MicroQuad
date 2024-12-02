#include "PersistentKeyValueStore.h"

#include <nvs.h>
#include <nvs_flash.h>

#include <string>

#include "Logger.h"

static const char *kTag = "PersistentKeyValueStore";
static const char *kNamespaceName = "kv_storage";

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

std::string PersistentKeyValueStore::getStringForKey(std::string key)
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

  char *value = new char[required_size];
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

int PersistentKeyValueStore::getIntForKey(std::string key)
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

float PersistentKeyValueStore::getFloatForKey(std::string key)
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

  char *str_value = new char[required_size];
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

void PersistentKeyValueStore::setStringForKey(std::string key, std::string value)
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

void PersistentKeyValueStore::setIntForKey(std::string key, int value)
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

void PersistentKeyValueStore::setFloatForKey(std::string key, float value)
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

bool PersistentKeyValueStore::hasValueForKey(std::string key)
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

void PersistentKeyValueStore::removeValueForKey(std::string key)
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