#pragma once

#include <queue>
#include <stdexcept>
#include <unordered_map>

template <typename T>
class MedianFilter
{
 public:
  explicit MedianFilter(size_t windowSize);

  void addValue(const T& value);

  T getMedian() const;

 private:
  size_t _windowSize;
  size_t _index;

  std::priority_queue<T> _maxHeap;                                   // Lower half
  std::priority_queue<T, std::vector<T>, std::greater<T>> _minHeap;  // Upper half

  std::unordered_map<T, size_t> _delayedElements;  // Elements to remove
  std::deque<std::pair<T, size_t>> _window;        // Sliding window with indices
};