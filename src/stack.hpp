#ifndef CONTAINERS_STACK_HPP
#define CONTAINERS_STACK_HPP

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <utility>

#include <memory>
#include <stdexcept>

namespace containers
{

template <typename T> class Stack final
{
  public:
    using value_type = T;

    Stack();
    explicit Stack(std::size_t initial_capacity);
    ~Stack() noexcept;

    Stack(const Stack &) = delete;
    Stack &operator=(const Stack &) = delete;
    Stack(Stack &&) = delete;
    Stack &operator=(Stack &&) = delete;

    void reserve(std::size_t new_capacity);
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    std::size_t capacity() const noexcept;
    void push(const T &value);
    void push(T &&value);
    template <typename... Args> void emplace(Args &&...args);
    void pop();
    bool try_pop() noexcept;
    T &top();
    const T &top() const;

  private:
    std::unique_ptr<T[]> buffer_;
    std::size_t size_;
    std::size_t capacity_;

    void resize(std::size_t new_capacity);
    void clear() noexcept;
};

template <typename T> Stack<T>::Stack() : Stack(1)
{
}

template <typename T>
Stack<T>::Stack(std::size_t initial_capacity)
    : buffer_(std::make_unique<T[]>(initial_capacity)), size_(0), capacity_(initial_capacity)
{
}

template <typename T> Stack<T>::~Stack() noexcept
{
    clear();
}

template <typename T> void Stack<T>::reserve(std::size_t new_capacity)
{
    resize(new_capacity);
}

template <typename T> bool Stack<T>::empty() const noexcept
{
    return size_ == 0;
}

template <typename T> std::size_t Stack<T>::size() const noexcept
{
    return size_;
}

template <typename T> std::size_t Stack<T>::capacity() const noexcept
{
    return capacity_;
}

template <typename T> void Stack<T>::push(const T &value)
{
    emplace(value);
}

template <typename T> void Stack<T>::push(T &&value)
{
    emplace(std::move(value));
}

template <typename T> template <typename... Args> void Stack<T>::emplace(Args &&...args)
{
    if (size_ >= capacity_)
    {
        resize(capacity_ * 2); // Double the capacity
    }
    ::new (&buffer_[size_]) T(std::forward<Args>(args)...);
    ++size_;
}

template <typename T> void Stack<T>::pop()
{
    if (empty())
    {
        throw std::out_of_range("Stack::pop(): stack is empty");
    }
    buffer_[size_ - 1].~T(); // Call destructor for the element
    --size_;
}

template <typename T> bool Stack<T>::try_pop() noexcept
{
    if (empty())
    {
        return false;
    }
    buffer_[size_ - 1].~T(); // Call destructor for the element
    --size_;
    return true;
}

template <typename T> T &Stack<T>::top()
{
    if (empty())
    {
        throw std::out_of_range("Stack::top(): stack is empty");
    }
    return buffer_[size_ - 1];
}

template <typename T> const T &Stack<T>::top() const
{
    if (empty())
    {
        throw std::out_of_range("Stack::top(): stack is empty");
    }
    return buffer_[size_ - 1];
}

template <typename T> void Stack<T>::resize(std::size_t new_capacity)
{
    if (new_capacity <= capacity_)
    {
        return; // Only resize if the new capacity is larger
    }

    auto new_buffer = std::make_unique<T[]>(new_capacity);

    for (std::size_t i = 0; i < size_; ++i)
    {
        ::new (&new_buffer[i]) T(std::move(buffer_[i]));

        buffer_[i].~T(); // Call destructor for the element
    }

    buffer_ = std::move(new_buffer);
    capacity_ = new_capacity;
}

template <typename T> void Stack<T>::clear() noexcept
{
    while (size_ > 0)
    {
        static_cast<void>(try_pop());
    }
}

} // namespace containers

#endif // CONTAINERS_STACK_HPP
