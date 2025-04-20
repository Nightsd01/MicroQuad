// AsyncController.cpp

#include "AsyncController.h"

// --- Common Includes (Needed by both ESP32 and Host) ---
#include <memory>

#include "Logger.h"

// --- Platform Specific Section ---

#ifndef MATLAB_SIM
// ========================================================================
// == ESP32 Implementation (using FreeRTOS)                            ==
// ========================================================================

#include "esp_timer.h"  // For millis() outside of arduino environment
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// --- ESP32 Configuration ---
#define DEFAULT_STACK_SIZE 4096
#define DEFAULT_TASK_PRIORITY 1

// --- ESP32 Helper Structs/Functions ---
namespace
{  // Use anonymous namespace for static helpers

struct _async_controller_timer_params_t
{
  AsyncController *self;
  std::function<void()> task;
};

void _handler(void *pvParams)
{
  // Safely capture and execute the task, then clean up
  std::unique_ptr<std::function<void()>> funcPtr(static_cast<std::function<void()> *>(pvParams));
  try {
    (*funcPtr)();
  } catch (const std::exception &e) {
    LOG_ERROR("Unhandled exception in FreeRTOS task: %s", e.what());
  } catch (...) {
    LOG_ERROR("Unknown unhandled exception in FreeRTOS task");
  }
  // funcPtr unique_ptr goes out of scope and deletes the function object
  vTaskDelete(NULL);  // Delete the task itself
}

void _timerCallback(TimerHandle_t xTimer)
{
  // Use unique_ptr to manage the params lifetime safely
  std::unique_ptr<_async_controller_timer_params_t> params(
      static_cast<_async_controller_timer_params_t *>(pvTimerGetTimerID(xTimer)));
  if (params) {
    params->self->executePossiblySync(
        params->task);  // Task ownership implicitly handled by lambda capture or std::function copy
  }
}

}  // end anonymous namespace

// --- ESP32 AsyncController Method Implementations ---

void AsyncController::executeAfter(uint64_t delayMillis, std::function<void()> task)
{
  // Use unique_ptr for exception safety when allocating params
  auto params_ptr = std::make_unique<_async_controller_timer_params_t>();
  if (!params_ptr) {
    LOG_ERROR("Failed to allocate memory for timer params");
    return;  // Early exit
  }
  // Populate params
  params_ptr->self = this;
  params_ptr->task = std::move(task);  // Transfer ownership

  TimerHandle_t timer = xTimerCreate(
      _taskLabel,                                        // Timer name (use task label for context)
      pdMS_TO_TICKS(delayMillis > 0 ? delayMillis : 1),  // Ensure ticks > 0
      pdFALSE,                                           // One-shot timer
      static_cast<void *>(
          params_ptr.get()),  // Pass raw pointer, ownership managed by unique_ptr until callback takes over
      _timerCallback);

  if (timer != nullptr) {
    if (xTimerStart(timer, 0) == pdPASS) {
      // Timer started successfully, release ownership from unique_ptr
      // as the timer system (via pvTimerGetTimerID in callback) is now responsible
      (void)params_ptr.release();
    } else {
      LOG_ERROR("Failed to start timer for %s", _taskLabel);
    }
  } else {
    LOG_ERROR("Failed to create timer for %s", _taskLabel);
  }
}

void AsyncController::_execute(std::function<void()> task, bool allowSync)
{
  // Use core ID for ESP32
  const int currentCore = xPortGetCoreID();
  if (currentCore == _sim_core_id_or_index && allowSync) {
    try {
      task();  // Execute synchronously
    } catch (const std::exception &e) {
      LOG_ERROR("Unhandled exception in sync task %s: %s", _taskLabel, e.what());
    } catch (...) {
      LOG_ERROR("Unknown unhandled exception in sync task %s", _taskLabel);
    }
    return;
  }

  // Execute asynchronously on the target core
  // Use unique_ptr for exception safety during allocation
  auto funcPtr = std::make_unique<std::function<void()>>(std::move(task));
  if (!funcPtr) {
    LOG_ERROR("Failed to allocate memory for async task %s", _taskLabel);
    return;  // Early exit
  }

  BaseType_t result = xTaskCreatePinnedToCore(
      _handler,                            // Task entry function
      _taskLabel,                          // Task name
      DEFAULT_STACK_SIZE,                  // Stack size
      static_cast<void *>(funcPtr.get()),  // Pass raw pointer
      DEFAULT_TASK_PRIORITY,               // Task priority
      NULL,                                // Task handle (optional)
      _sim_core_id_or_index                // Core ID (0 or 1)
  );

  if (result == pdPASS) {
    // Task created successfully, release ownership from unique_ptr
    // The _handler function is now responsible for deleting it
    (void)funcPtr.release();
  } else {
    // Handle task creation failure
    LOG_ERROR("Failed to create async task %s (Error code: %ld)", _taskLabel, result);
    // funcPtr goes out of scope here, cleaning up the function object automatically
  }
}

#else  // MATLAB_SIM
// ========================================================================
// == Host Implementation (using C++ Standard Library)                 ==
// ========================================================================

#include <future>    // For std::async
#include <iostream>  // For std::cerr fallback logging
#include <thread>    // For std::this_thread::sleep_for

// --- Host Configuration / Fallbacks ---
#ifndef LOG_ERROR
#include <cstdio>
#define LOG_ERROR(format, ...) fprintf(stderr, "[ERROR] AsyncController: " format "\n", ##__VA_ARGS__)
#endif

// --- Host AsyncController Method Implementations ---

void AsyncController::executeAfter(uint64_t delayMillis, std::function<void()> task)
{
  // Use std::async to launch a task that sleeps then executes
  // The returned future is intentionally ignored (fire-and-forget)
  std::async(std::launch::async, [delayMillis, task_copy = std::move(task), label = _taskLabel]() {
    try {
      if (delayMillis > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMillis));
      }
      task_copy();
    } catch (const std::exception &e) {
      LOG_ERROR("Task exception in executeAfter (%s): %s", label, e.what());
    } catch (...) {
      LOG_ERROR("Unknown task exception in executeAfter (%s)", label);
    }
  });
}

void AsyncController::_execute(std::function<void()> task, bool allowSync)
{
  // In host mode, we ignore core affinity (_sim_core_id_or_index)
  // The decision is purely based on allowSync.
  if (allowSync) {
    try {
      task();  // Execute synchronously on the calling thread
    } catch (const std::exception &e) {
      LOG_ERROR("Sync task exception (%s): %s", _taskLabel, e.what());
    } catch (...) {
      LOG_ERROR("Unknown sync task exception (%s)", _taskLabel);
    }
  } else {
    // Execute asynchronously using std::async
    // The returned future is intentionally ignored (fire-and-forget)
    std::async(std::launch::async, [task_copy = std::move(task), label = _taskLabel]() {
      try {
        task_copy();
      } catch (const std::exception &e) {
        LOG_ERROR("Async task exception (%s): %s", label, e.what());
      } catch (...) {
        LOG_ERROR("Unknown async task exception (%s)", label);
      }
    });
  }
}

#endif  // MATLAB_SIM

// ========================================================================
// == Common Code (Used by both ESP32 and Host implementations)        ==
// ========================================================================

// Define the static member variables (definitions are common)
// The constructor logic is simple enough to be common.
// The 'core' parameter's meaning differs slightly by platform but storage is the same.
AsyncController AsyncController::main(1, "main_task");      // ESP Core 1 / Host Index 1
AsyncController AsyncController::background(0, "bg_task");  // ESP Core 0 / Host Index 0

// Constructor
AsyncController::AsyncController(int sim_core_id_or_index, const char *taskLabel)
    : _sim_core_id_or_index(sim_core_id_or_index), _taskLabel(taskLabel)
{
}

// Public interface methods often just delegate
void AsyncController::executePossiblySync(std::function<void()> task)
{
  // Move task into the potentially platform-specific implementation
  this->_execute(std::move(task), true /* allowSync */);
}

void AsyncController::executeAsync(std::function<void()> task)
{
  // Move task into the potentially platform-specific implementation
  this->_execute(std::move(task), false /* allowSync */);
}

// executeOnce uses std::call_once which is portable C++11
void AsyncController::executeOnce(AsyncControllerOnceToken &flag, std::function<void()> task)
{
  // Ensures the lambda (which captures 'this' and 'task') is called only once
  // Use mutable lambda in case the task itself needs to modify its captured state (though unlikely here)
  std::call_once(flag, [this, task_copy = std::move(task)]() mutable {
    // Move task copy into the execution function
    this->executePossiblySync(std::move(task_copy));  // Use PossiblySync for execution
  });
}