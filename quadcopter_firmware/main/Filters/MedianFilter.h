#pragma once

#include <cstddef>  // for size_t
#include <deque>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <utility>

template <typename T>
class MedianFilter
{
 public:
  explicit MedianFilter(size_t windowSize);

  void addValue(const T& value);
  T getMedian() const;

 private:
  void rebalanceHeaps();
  void removeDelayedElements(std::priority_queue<std::pair<T, size_t>>& heap, bool isMaxHeap);
  void removeDelayedElements(
      std::priority_queue<std::pair<T, size_t>, std::vector<std::pair<T, size_t>>, std::greater<std::pair<T, size_t>>>&
          heap,
      bool isMaxHeap);

  size_t _windowSize;
  size_t _index;

  // Heaps store (value, index) pairs.
  std::priority_queue<std::pair<T, size_t>> _maxHeap;
  std::priority_queue<std::pair<T, size_t>, std::vector<std::pair<T, size_t>>, std::greater<std::pair<T, size_t>>>
      _minHeap;

  // Track indices of elements that are outside the window and should be removed
  std::unordered_set<size_t> _delayedElements;

  // Sliding window to know which element (value,index) to remove next
  std::deque<std::pair<T, size_t>> _window;
};