#ifndef COMMON_LIBRARY_CONTAINERS_BOUNDED_DYNAMIC_ARRAY
#define COMMON_LIBRARY_CONTAINERS_BOUNDED_DYNAMIC_ARRAY

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

namespace lidar_processing::utilities
{
template <typename DataType, std::size_t MaxSize, bool SafeMode = false>
class BoundedDynamicArray
{
  public:
    static constexpr auto MAX_SIZE = MaxSize;
    static constexpr auto SAFE_MODE = SafeMode;

    BoundedDynamicArray()
        : data_{std::make_unique<DataType[]>(MAX_SIZE)}, size_{0U},
          capacity_{MAX_SIZE}
    {
    }

    // Element access
    inline DataType& operator[](const std::size_t i)
    {
        if constexpr (SAFE_MODE)
        {
            if (i >= size_)
            {
                throw std::out_of_range("Access out of bounds");
            }
        }
        return data_[i];
    }
    inline const DataType& operator[](const std::size_t i) const
    {
        if constexpr (SAFE_MODE)
        {
            if (i >= size_)
            {
                throw std::out_of_range("Access out of bounds");
            }
        }
        return data_[i];
    }

    // Iterators
    inline DataType* begin() noexcept
    {
        return data_.get();
    }
    inline const DataType* begin() const noexcept
    {
        return data_.get();
    }
    inline DataType* end() noexcept
    {
        return data_.get() + size_;
    }
    inline const DataType* end() const noexcept
    {
        return data_.get() + size_;
    }

    // Capacity
    inline std::size_t size() const noexcept
    {
        return size_;
    }
    inline std::size_t capacity() const noexcept
    {
        return capacity_;
    }

    // Modifiers
    inline void push_back(const DataType& value)
    {
        if constexpr (SAFE_MODE)
        {
            if (size_ >= capacity_)
            {
                throw std::length_error("Exceeded capacity");
            }
        }
        data_[size_++] = value;
    }
    inline void push_back(DataType&& value)
    {
        if constexpr (SAFE_MODE)
        {
            if (size_ >= capacity_)
            {
                throw std::length_error("Exceeded capacity");
            }
        }
        data_[size_++] = std::move(value);
    }
    inline void pop_back() noexcept
    {
        if (size_ > 0U)
        {
            --size_;
        }
    }
    inline void resize(const std::size_t new_size)
    {
        if constexpr (SAFE_MODE)
        {
            if (new_size > capacity_)
            {
                throw std::length_error("Exceeded capacity");
            }
        }
        size_ = new_size;
    }
    inline void clear() noexcept
    {
        size_ = 0U;
    }

  private:
    std::unique_ptr<DataType[]> data_;
    std::size_t size_;
    std::size_t capacity_;
};
} // namespace lidar_processing::utilities

#endif // COMMON_LIBRARY_CONTAINERS_BOUNDED_DYNAMIC_ARRAY