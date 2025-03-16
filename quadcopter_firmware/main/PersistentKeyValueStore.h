#pragma once

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

  void setStringForKey(const std::string& key, const std::string& value);
  void setIntForKey(const std::string& key, int value);
  void setFloatForKey(const std::string& key, float value);

  bool hasValueForKey(const std::string& key);
  void removeValueForKey(const std::string& key);
};