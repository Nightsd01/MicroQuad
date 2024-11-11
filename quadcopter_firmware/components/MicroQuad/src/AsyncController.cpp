#include "AsyncController.h"

#include "Logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define DEFAULT_STACK_SIZE 4096
#define DEFAULT_TASK_PRIORITY 1

struct _async_controller_timer_params_t
{
  AsyncController *self;
  std::function<void()> task;
};

// Define the static member variables
AsyncController AsyncController::main(1, "main_task");
AsyncController AsyncController::background(0, "bg_task");

static void _handler(void *pvParams)
{
  std::function<void()> *funcPtr = static_cast<std::function<void()> *>(pvParams);
  (*funcPtr)();
  delete funcPtr;
  vTaskDelete(NULL);
}

static void _timerCallback(TimerHandle_t xTimer)
{
  auto *params = static_cast<_async_controller_timer_params_t *>(pvTimerGetTimerID(xTimer));
  params->self->executePossiblySync(params->task);
  free(params);
}

// Public Functions
AsyncController::AsyncController(bool core, const char *taskLabel)
{
  _core = core;
  _taskLabel = taskLabel;
}

void AsyncController::executePossiblySync(std::function<void()> task) { this->_execute(task, true /* allowSync */); }

void AsyncController::executeAsync(std::function<void()> task) { this->_execute(task, false /* allowSync */); }

void AsyncController::executeOnce(AsyncControllerOnceToken &flag, std::function<void()> task)
{
  std::call_once(flag, [this, task = std::move(task)]() { this->executePossiblySync(task); });
}

void AsyncController::executeAfter(uint64_t delayMillis, std::function<void()> task)
{
  auto *params = new _async_controller_timer_params_t{
      .self = this,  // if calling from within AsyncController
      .task = std::move(task)};
  if (params == nullptr) {
    LOG_ERROR("Failed to allocate memory for timer params");
    return;
  }
  TimerHandle_t timer = xTimerCreate(
      "DelayedExecutor",
      pdMS_TO_TICKS(delayMillis),
      pdFALSE,  // One-shot
      static_cast<void *>(params),
      _timerCallback);
  if (timer != nullptr) {
    xTimerStart(timer, 0);
  } else {
    LOG_ERROR("Failed to create timer");
  }
}

// Private Functions
void AsyncController::_execute(std::function<void()> task, bool allowSync)
{
  const int currentCore = xPortGetCoreID();
  if (currentCore == _core && allowSync) {
    task();
    return;
  }

  std::function<void()> *funcPtr = new std::function<void()>(std::move(task));
  BaseType_t result = xTaskCreatePinnedToCore(
      _handler,               // Task entry function
      _taskLabel,             // Task name
      DEFAULT_STACK_SIZE,     // Stack size in bytes
      funcPtr,                // Parameter passed to the task
      DEFAULT_TASK_PRIORITY,  // Task priority
      NULL,                   // Task handle (optional)
      _core                   // Core ID (0 or 1)
  );

  if (result != pdPASS) {
    // Handle task creation failure
    delete funcPtr;  // Clean up allocated memory
    LOG_ERROR("Failed to create task %s", _taskLabel);
  }
}