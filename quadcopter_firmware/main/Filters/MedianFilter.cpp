#include "MedianFilter.h"

#include "esp_err.h"

template <typename T>
MedianFilter<T>::MedianFilter(size_t windowSize) : _windowSize(windowSize), _index(0)
{
  if (_windowSize == 0) {
    ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
  }
}

template <typename T>
void MedianFilter<T>::addValue(const T& value)
{
  // Add new value to the appropriate heap
  if (_maxHeap.empty() || value <= _maxHeap.top()) {
    _maxHeap.push(value);
  } else {
    _minHeap.push(value);
  }

  // Add the value and its index to the window
  _window.emplace_back(value, _index);

  // Rebalance the heaps if necessary
  if (_maxHeap.size() > _minHeap.size() + 1) {
    _minHeap.push(_maxHeap.top());
    _maxHeap.pop();
  } else if (_minHeap.size() > _maxHeap.size() + 1) {
    _maxHeap.push(_minHeap.top());
    _minHeap.pop();
  }

  // Remove elements outside the window
  if (_window.size() > _windowSize) {
    T old_value = _window.front().first;
    size_t old_index = _window.front().second;
    _window.pop_front();

    // Mark the old value for delayed removal
    _delayedElements[old_value]++;

    // Remove outdated elements from maxHeap
    while (!_maxHeap.empty() && _delayedElements.count(_maxHeap.top())) {
      _delayedElements[_maxHeap.top()]--;
      if (_delayedElements[_maxHeap.top()] == 0) {
        _delayedElements.erase(_maxHeap.top());
      }
      _maxHeap.pop();
    }

    // Remove outdated elements from minHeap
    while (!_minHeap.empty() && _delayedElements.count(_minHeap.top())) {
      _delayedElements[_minHeap.top()]--;
      if (_delayedElements[_minHeap.top()] == 0) {
        _delayedElements.erase(_minHeap.top());
      }
      _minHeap.pop();
    }
  }

  ++_index;
}

template <typename T>
T MedianFilter<T>::getMedian() const
{
  if (_maxHeap.empty() && _minHeap.empty()) throw std::runtime_error("No data available to compute median.");

  if (_maxHeap.size() == _minHeap.size()) {
    // Even number of elements
    return static_cast<T>((_maxHeap.top() + _minHeap.top()) / static_cast<double>(2));
  } else if (_maxHeap.size() > _minHeap.size()) {
    // maxHeap has more elements
    return _maxHeap.top();
  } else {
    // minHeap has more elements
    return _minHeap.top();
  }
}

// Add additional templates as needed for additional types
template class MedianFilter<int16_t>;
template class MedianFilter<float>;