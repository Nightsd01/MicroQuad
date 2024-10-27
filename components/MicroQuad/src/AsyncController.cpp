#include "AsyncController.h"

#include "Logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DEFAULT_STACK_SIZE 4096
#define DEFAULT_TASK_PRIORITY 1

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

AsyncController::AsyncController(
    bool core,
    const char *taskLabel)
{
  _core = core;
  _taskLabel = taskLabel;
}

void AsyncController::execute(std::function<void()> task)
{
  std::function<void()> *funcPtr = new std::function<void()>(std::move(task));
  BaseType_t result = xTaskCreatePinnedToCore(
      _handler,              // Task entry function
      _taskLabel,            // Task name
      DEFAULT_STACK_SIZE,    // Stack size in bytes
      funcPtr,               // Parameter passed to the task
      DEFAULT_TASK_PRIORITY, // Task priority
      NULL,                  // Task handle (optional)
      _core                  // Core ID (0 or 1)
  );

  if (result != pdPASS)
  {
    // Handle task creation failure
    delete funcPtr; // Clean up allocated memory
    LOG_ERROR("Failed to create task %s", _taskLabel);
  }
}