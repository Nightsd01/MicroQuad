#include "MedianFilter.h"

#ifndef MATLAB_SIM
#include "esp_err.h"
#endif  // MATLAB_SIM

template <typename T>
MedianFilter<T>::MedianFilter(size_t windowSize) : _windowSize(windowSize), _index(0)
{
  if (_windowSize == 0) {
#ifndef MATLAB_SIM
    ESP_ERROR_CHECK(ESP_ERR_INVALID_ARG);
#else
    throw std::invalid_argument("Window size must be greater than 0");
#endif  // MATLAB_SIM
  }
}

template <typename T>
void MedianFilter<T>::addValue(const T& value)
{
  // Insert (value, index) into one of the heaps
  if (_maxHeap.empty() || value <= getMaxHeapTop()) {
    _maxHeap.push(std::make_pair(value, _index));
  } else {
    _minHeap.push(std::make_pair(value, _index));
  }

  _window.emplace_back(value, _index);

  // If the window is too large, remove the oldest element
  if (_window.size() > _windowSize) {
    auto old = _window.front();
    _window.pop_front();

    // Mark old element for delayed removal
    _delayedElements.insert(old.second);
  }

  // Clean up delayed elements and rebalance
  cleanupAndRebalance();

  ++_index;
}

template <typename T>
T MedianFilter<T>::getMedian() const
{
  if (getValidMaxHeapSize() == 0 && getValidMinHeapSize() == 0) {
    throw std::runtime_error("No data available to compute median.");
  }

  size_t maxSize = getValidMaxHeapSize();
  size_t minSize = getValidMinHeapSize();

  // If sizes are equal, median is average of tops
  if (maxSize == minSize) {
    double val = (static_cast<double>(getMaxHeapTop()) + static_cast<double>(getMinHeapTop())) / 2.0;
    return static_cast<T>(val);
  } else if (maxSize > minSize) {
    return getMaxHeapTop();
  } else {
    return getMinHeapTop();
  }
}

template <typename T>
void MedianFilter<T>::cleanupAndRebalance()
{
  // Clean up delayed elements from both heaps
  removeDelayedElements(_maxHeap, true);
  removeDelayedElements(_minHeap, false);

  // Rebalance heaps to maintain median property
  rebalanceHeaps();
}

template <typename T>
void MedianFilter<T>::rebalanceHeaps()
{
  size_t maxSize = getValidMaxHeapSize();
  size_t minSize = getValidMinHeapSize();

  // Ensure heaps differ in size by at most 1
  while (maxSize > minSize + 1) {
    // Move from max heap to min heap
    auto top = getMaxHeapTop();
    size_t topIndex = getMaxHeapTopIndex();
    _maxHeap.pop();
    removeDelayedElements(_maxHeap, true);  // Clean up after pop
    
    _minHeap.push(std::make_pair(top, topIndex));
    
    maxSize = getValidMaxHeapSize();
    minSize = getValidMinHeapSize();
  }
  
  while (minSize > maxSize + 1) {
    // Move from min heap to max heap
    auto top = getMinHeapTop();
    size_t topIndex = getMinHeapTopIndex();
    _minHeap.pop();
    removeDelayedElements(_minHeap, false);  // Clean up after pop
    
    _maxHeap.push(std::make_pair(top, topIndex));
    
    maxSize = getValidMaxHeapSize();
    minSize = getValidMinHeapSize();
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

template <typename T>
T MedianFilter<T>::getMaxHeapTop() const
{
  // Clean up delayed elements first
  while (!_maxHeap.empty() && _delayedElements.find(_maxHeap.top().second) != _delayedElements.end()) {
    const_cast<MedianFilter<T>*>(this)->_delayedElements.erase(_maxHeap.top().second);
    const_cast<MedianFilter<T>*>(this)->_maxHeap.pop();
  }
  
  if (_maxHeap.empty()) {
    throw std::runtime_error("Max heap is empty");
  }
  
  return _maxHeap.top().first;
}

template <typename T>
T MedianFilter<T>::getMinHeapTop() const
{
  // Clean up delayed elements first
  while (!_minHeap.empty() && _delayedElements.find(_minHeap.top().second) != _delayedElements.end()) {
    const_cast<MedianFilter<T>*>(this)->_delayedElements.erase(_minHeap.top().second);
    const_cast<MedianFilter<T>*>(this)->_minHeap.pop();
  }
  
  if (_minHeap.empty()) {
    throw std::runtime_error("Min heap is empty");
  }
  
  return _minHeap.top().first;
}

template <typename T>
size_t MedianFilter<T>::getMaxHeapTopIndex() const
{
  // Clean up delayed elements first
  while (!_maxHeap.empty() && _delayedElements.find(_maxHeap.top().second) != _delayedElements.end()) {
    const_cast<MedianFilter<T>*>(this)->_delayedElements.erase(_maxHeap.top().second);
    const_cast<MedianFilter<T>*>(this)->_maxHeap.pop();
  }
  
  if (_maxHeap.empty()) {
    throw std::runtime_error("Max heap is empty");
  }
  
  return _maxHeap.top().second;
}

template <typename T>
size_t MedianFilter<T>::getMinHeapTopIndex() const
{
  // Clean up delayed elements first
  while (!_minHeap.empty() && _delayedElements.find(_minHeap.top().second) != _delayedElements.end()) {
    const_cast<MedianFilter<T>*>(this)->_delayedElements.erase(_minHeap.top().second);
    const_cast<MedianFilter<T>*>(this)->_minHeap.pop();
  }
  
  if (_minHeap.empty()) {
    throw std::runtime_error("Min heap is empty");
  }
  
  return _minHeap.top().second;
}

template <typename T>
size_t MedianFilter<T>::getValidMaxHeapSize() const
{
  // Count valid elements (not marked for delayed removal)
  size_t count = 0;
  auto tempHeap = _maxHeap;
  
  while (!tempHeap.empty()) {
    if (_delayedElements.find(tempHeap.top().second) == _delayedElements.end()) {
      count++;
    }
    tempHeap.pop();
  }
  
  return count;
}

template <typename T>
size_t MedianFilter<T>::getValidMinHeapSize() const
{
  // Count valid elements (not marked for delayed removal)
  size_t count = 0;
  auto tempHeap = _minHeap;
  
  while (!tempHeap.empty()) {
    if (_delayedElements.find(tempHeap.top().second) == _delayedElements.end()) {
      count++;
    }
    tempHeap.pop();
  }
  
  return count;
}

// Explicit template instantiations as needed
template class MedianFilter<int16_t>;
template class MedianFilter<float>;