#ifndef ASYNC_CONTROLLER_H
#define ASYNC_CONTROLLER_H

#include <functional>
#include <mutex>

#define AsyncControllerOnceToken std::once_flag

class AsyncController
{
 public:
  // Static member variables
  static AsyncController main;
  static AsyncController background;
  void executePossiblySync(std::function<void()> task);
  void executeAsync(std::function<void()> task);

  // Guaranteed to execute only once per the given flag
  // If you are currently running on main and try to call this
  // from bg (or visa versa), this will be an async call
  // Otherwise it will be synchronous
  void executeOnce(AsyncControllerOnceToken &flag, std::function<void()> task);

 private:
  AsyncController(bool core, const char *taskLabel);
  void _execute(std::function<void()> task, bool allowSync);
  bool _core;
  const char *_taskLabel;
};

#endif