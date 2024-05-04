#ifndef CONTAINERS_PRIORITY_QUEUE_HPP
#define CONTAINERS_PRIORITY_QUEUE_HPP

#include "vector.hpp"

// STL
#include <algorithm>
#include <functional>

namespace containers
{
template <typename T, typename Comparator = std::less<T>> class PriorityQueue final
{
  public:
    using value_type = T;

    explicit PriorityQueue(Comparator comp = Comparator());

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    void push(const T &value);
    void push(T &&value);
    template <typename... Args> void emplace(Args &&...args);
    void pop();
    bool try_pop() noexcept;
    T &top();
    const T &top() const;

  private:
    containers::Vector<T> buffer_;
    Comparator comp_;
};

template <typename T, typename Comparator> PriorityQueue<T, Comparator>::PriorityQueue(Comparator comp) : comp_(comp)
{
    std::make_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator> void PriorityQueue<T, Comparator>::reserve(std::size_t new_capacity)
{
    buffer_.reserve(new_capacity);
}

template <typename T, typename Comparator> bool PriorityQueue<T, Comparator>::empty() const noexcept
{
    return buffer_.empty();
}

template <typename T, typename Comparator> std::size_t PriorityQueue<T, Comparator>::size() const noexcept
{
    return buffer_.size();
}

template <typename T, typename Comparator> void PriorityQueue<T, Comparator>::push(const T &value)
{
    buffer_.push_back(value);
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator> void PriorityQueue<T, Comparator>::push(T &&value)
{
    buffer_.push_back(std::move(value));
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator>
template <typename... Args>
void PriorityQueue<T, Comparator>::emplace(Args &&...args)
{
    buffer_.emplace_back(std::forward<Args>(args)...);
    std::push_heap(buffer_.begin(), buffer_.end(), comp_);
}

template <typename T, typename Comparator> void PriorityQueue<T, Comparator>::pop()
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::pop(): queue is empty");
    }
    std::pop_heap(buffer_.begin(), buffer_.end(), comp_);
    buffer_.pop_back();
}

template <typename T, typename Comparator> bool PriorityQueue<T, Comparator>::try_pop() noexcept
{
    if (empty())
    {
        return false;
    }
    std::pop_heap(buffer_.begin(), buffer_.end(), comp_);
    buffer_.pop_back();
}

template <typename T, typename Comparator> T &PriorityQueue<T, Comparator>::top()
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::top(): queue is empty");
    }
    return buffer_.front();
}

template <typename T, typename Comparator> const T &PriorityQueue<T, Comparator>::top() const
{
    if (empty())
    {
        throw std::out_of_range("PriorityQueue::top(): queue is empty");
    }
    return buffer_.front();
}

} // namespace containers

#endif // CONTAINERS_PRIORITY_QUEUE_HPP
