#ifndef BOUNDED_VECTOR_HPP
#define BOUNDED_VECTOR_HPP

#include <algorithm>        // std::copy, std::swap
#include <cstdint>          // std::size_t
#include <initializer_list> // std::initializer_list
#include <stdexcept>        // std::out_of_range

namespace lidar_processing::utilities
{
template <typename T, std::size_t Size> class BoundedVector
{
  public:
    using iterator = T *;
    using const_iterator = const T *;

    BoundedVector(BoundedVector &&other) = delete;
    BoundedVector &operator=(BoundedVector &&other) = delete;
    BoundedVector(const BoundedVector &other) = delete;
    BoundedVector &operator=(const BoundedVector &other) = delete;

    BoundedVector() : size_{0U}
    {
        buffer_ = new T[Size];
    }

    ~BoundedVector()
    {
        delete[] buffer_;
    }

    iterator begin() noexcept
    {
        return buffer_;
    }

    const_iterator begin() const noexcept
    {
        return buffer_;
    }

    iterator end() noexcept
    {
        return buffer_ + size_;
    }

    const_iterator end() const noexcept
    {
        return buffer_ + size_;
    }

    bool push_back(const T &value) noexcept
    {
        if (size_ >= Size)
        {
            return false;
        }
        buffer_[size_++] = value;
        return true;
    }

    bool push_back(T &&value) noexcept
    {
        if (size_ >= Size)
        {
            return false;
        }
        buffer_[size_++] = std::move(value);
        return true;
    }

    bool pop_back() noexcept
    {
        if (size_ <= 0)
        {
            return false;
        }
        --size_;
        return true;
    }

    T &operator[](std::size_t index)
    {
        return buffer_[index];
    }

    const T &operator[](std::size_t index) const
    {
        return buffer_[index];
    }

    T &at(std::size_t index)
    {
        if (index >= size_)
        {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[index];
    }

    const T &at(std::size_t index) const
    {
        if (index >= size_)
        {
            throw std::out_of_range("Index out of range");
        }
        return buffer_[index];
    }

    std::size_t size() const noexcept
    {
        return size_;
    }

  private:
    std::size_t size_;
    T buffer_;
};
} // namespace lidar_processing::utilities

#endif