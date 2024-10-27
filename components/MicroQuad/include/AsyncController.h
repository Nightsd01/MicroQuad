#ifndef ASYNC_CONTROLLER_H
#define ASYNC_CONTROLLER_H

#include <functional>

class AsyncController {
 public:
  // Static member variables
  static AsyncController main;
  static AsyncController background;
  void execute(std::function<void()> task);

 private:
  AsyncController(bool core, const char *taskLabel);
  bool _core;
  const char *_taskLabel;
};

#endif