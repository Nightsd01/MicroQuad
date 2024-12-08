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
  // Insert (value, index) into one of the heaps
  if (_maxHeap.empty() || value <= _maxHeap.top().first) {
    _maxHeap.push(std::make_pair(value, _index));
  } else {
    _minHeap.push(std::make_pair(value, _index));
  }

  _window.emplace_back(value, _index);

  // Balance after adding the new element
  rebalanceHeaps();

  // If the window is too large, remove the oldest element
  if (_window.size() > _windowSize) {
    auto old = _window.front();
    _window.pop_front();

    // Mark old element for delayed removal
    _delayedElements.insert(old.second);

    // Clean up any delayed elements now at the top
    removeDelayedElements(_maxHeap, true);
    removeDelayedElements(_minHeap, false);

    // Rebalance again if needed after removals
    rebalanceHeaps();
  }

  ++_index;
}

template <typename T>
T MedianFilter<T>::getMedian() const
{
  if (_maxHeap.empty() && _minHeap.empty()) {
    throw std::runtime_error("No data available to compute median.");
  }

  // If sizes are equal, median is average of tops
  if (_maxHeap.size() == _minHeap.size()) {
    double val = (static_cast<double>(_maxHeap.top().first) + static_cast<double>(_minHeap.top().first)) / 2.0;
    return static_cast<T>(val);
  } else if (_maxHeap.size() > _minHeap.size()) {
    return _maxHeap.top().first;
  } else {
    return _minHeap.top().first;
  }
}

template <typename T>
void MedianFilter<T>::rebalanceHeaps()
{
  // Ensure heaps differ in size by at most 1
  if (_maxHeap.size() > _minHeap.size() + 1) {
    _minHeap.push(_maxHeap.top());
    _maxHeap.pop();
  } else if (_minHeap.size() > _maxHeap.size() + 1) {
    _maxHeap.push(_minHeap.top());
    _minHeap.pop();
  }
}

template <typename T>
void MedianFilter<T>::removeDelayedElements(std::priority_queue<std::pair<T, size_t>>& heap, bool isMaxHeap)
{
  // Remove elements from the heap top that are marked for delayed removal
  while (!heap.empty() && _delayedElements.find(heap.top().second) != _delayedElements.end()) {
    _delayedElements.erase(heap.top().second);
    heap.pop();
  }
}

template <typename T>
void MedianFilter<T>::removeDelayedElements(
    std::priority_queue<std::pair<T, size_t>, std::vector<std::pair<T, size_t>>, std::greater<std::pair<T, size_t>>>&
        heap,
    bool isMaxHeap)
{
  // Remove elements from the heap top that are marked for delayed removal
  while (!heap.empty() && _delayedElements.find(heap.top().second) != _delayedElements.end()) {
    _delayedElements.erase(heap.top().second);
    heap.pop();
  }
}

// Explicit template instantiations as needed
template class MedianFilter<int16_t>;
template class MedianFilter<float>;