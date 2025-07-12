#pragma once

#ifndef MATLAB_SIM

#include <array>
#include <concepts>
#include <string>
#include <type_traits>
#include <vector>

class PersistentKeyValueStore
{
 public:
  PersistentKeyValueStore();

  // Generic getter for all types
  template <typename T>
  T getValue(const std::string& key);

  // Special overload for vectors that need a size parameter
  template <typename T>
  std::vector<T> getValue(const std::string& key, size_t length);

  // Generic setter for all types
  template <typename T>
  void setValue(const std::string& key, const T& value);

  // Utility methods
  bool hasValueForKey(const std::string& key);
  void removeValueForKey(const std::string& key);
};

#endif  // MATLAB_SIM