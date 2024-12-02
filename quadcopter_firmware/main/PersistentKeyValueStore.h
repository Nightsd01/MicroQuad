#pragma once

#include <string>

class PersistentKeyValueStore
{
 public:
  PersistentKeyValueStore();

  std::string getStringForKey(std::string key);
  int getIntForKey(std::string key);
  float getFloatForKey(std::string key);

  void setStringForKey(std::string key, std::string value);
  void setIntForKey(std::string key, int value);
  void setFloatForKey(std::string key, float value);

  bool hasValueForKey(std::string key);
  void removeValueForKey(std::string key);
};