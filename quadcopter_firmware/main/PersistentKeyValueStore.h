#pragma once

#ifndef MATLAB_SIM

#include <array>
#include <string>
#include <vector>

class PersistentKeyValueStore
{
 public:
  PersistentKeyValueStore();

  std::string getStringForKey(const std::string& key);
  int getIntForKey(const std::string& key);
  float getFloatForKey(const std::string& key);
  template <typename T>
  std::vector<T> getVectorForKey(const std::string& key, size_t length);

  void setStringForKey(const std::string& key, const std::string& value);
  void setIntForKey(const std::string& key, int value);
  void setFloatForKey(const std::string& key, float value);
  template <typename T>
  void setVectorForKey(const std::string& key, const std::vector<T>& value);

  bool hasValueForKey(const std::string& key);
  void removeValueForKey(const std::string& key);
};

#endif  // MATLAB_SIM