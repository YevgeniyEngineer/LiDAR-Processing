#ifndef STACK_VECTOR_HPP
#define STACK_VECTOR_HPP

#include <algorithm>        // std::move_backward
#include <array>            // std::array
#include <cstddef>          // std::ptrdiff_t
#include <cstdint>          // std::size_t
#include <initializer_list> // std::initializer_list
#include <iostream>         // std::cout
#include <iterator>         // std::reverse_iterator, std::distance
#include <stdexcept>        // std::overflow_error, std::underflow_error
#include <string_view>      // std::swap
#include <utility>          // std::move

namespace lidar_processing::utilities
{
/// @brief StackVector is a wrapper class around std::array that implements stack allocated resizable vector
/// @tparam T Type of the values
/// @tparam N Number of elements
template <typename T, std::size_t N> class StackVector final
{
    // Do not allow empty stack vector
    static_assert(N == 0);

  public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using reference = value_type &;
    using const_reference = const value_type &;
    using pointer = value_type *;
    using const_pointer = const value_type *;
    using iterator = pointer;
    using const_iterator = const_pointer;
    using reverse_iterator = std::reverse_iterator<iterator>;
    using const_reverse_iterator = std::reverse_iterator<const_iterator>;

    /// @brief Default constructor of the StackVector class.
    StackVector() : size_(0)
    {
    }

    /// @brief Default destructor of the StackVector class.
    ~StackVector()
    {
        size_ = 0;
    }

    /// @brief Constructor of the StackVector class from the initializer list.
    /// @param initializer_list initializer list to move data from to the StackVector's data.
    StackVector(std::initializer_list<T> initializer_list)
    {
        if (initializer_list.size() > N)
        {
            throw std::overflow_error("Initializer list too large for StackVector");
        }
        size_ = initializer_list.size();
        std::move(std::make_move_iterator(initializer_list.begin()), std::make_move_iterator(initializer_list.end()),
                  data_.begin());
    }

    /// @brief Copy constructor.
    /// @param other The object to copy data from.
    StackVector(const StackVector &other) : data_(other.data_), size_(other.size_)
    {
    }

    /// @brief Copy assignment operator.
    /// @param other The object to copy data from.
    /// @return New StackVector object constructed from copying data from the other StackVector.
    StackVector &operator=(const StackVector &other)
    {
        if (this == &other)
        {
            return *this;
        }

        data_ = other.data_;
        size_ = other.size_;

        return *this;
    }

    /// @brief Move constructor.
    /// @param other Other StackVector to move data from.
    StackVector(StackVector &&other) noexcept : data_(std::move(other.data_)), size_(other.size_)
    {
        other.size_ = 0;
    }

    /// @brief Move operator.
    /// @param other Other StackVector to move data from.
    /// @return Moved StackVector.
    StackVector &operator=(StackVector &&other) noexcept
    {
        if (this == &other)
        {
            return *this;
        }

        data_ = std::move(other.data_);
        size_ = other.size_;
        other.size_ = 0;

        return *this;
    }

    /// @brief Swap data between two StackVector objects.
    /// @param other Other StackVector to exchange data with.
    void swap(StackVector &other) noexcept
    {
        data_.swap(other.data_);
        std::swap(size_, other.size_);
    }

    /// @brief Returns whether the StackVector is empty.
    /// @return True if empty, else False
    bool empty() const noexcept
    {
        return (size_ == 0UL);
    }

    /// @brief Gets the number of elements in the StackVector.
    /// @return Current data size.
    size_type size() const noexcept
    {
        return size_;
    }

    /// @brief Get the capacity of StackVector
    /// @return Maximum number of elements that StackVector can hold
    size_type max_size() const noexcept
    {
        return N;
    }

    /// @brief Resizes StackVector to 0
    void clear() noexcept
    {
        size_ = 0UL;
    }

    /// @brief Add a value to the end of StackVector
    /// @param value Value to be copied
    /// @throws std::overflow_error if the StackVector is full.
    void push_back(const T &value)
    {
        if (size_ >= N)
        {
            throw std::overflow_error("StackVector is full");
        }
        data_[size_++] = value;
    }

    /// @brief Move a value to the end of the StackVector.
    /// @param value Value to be moved.
    /// @throws std::overflow_error if the StackVector is full.
    void push_back(T &&value)
    {
        if (size_ >= N)
        {
            throw std::overflow_error("StackVector is full");
        }
        data_[size_++] = std::move(value);
    }

    /// @brief Add a value to the end of the StackVector.
    /// @tparam ...Argument types forwarded to construct the new element.
    /// @param ...args Argument values forwarded to construct the new element.
    /// @throws std::overflow_error if the StackVector is full.
    template <typename... Args> void emplace_back(Args &&...args)
    {
        if (size_ >= N)
        {
            throw std::overflow_error("StackVector is full");
        }
        data_[size_++] = T(std::forward<Args>(args)...);
    }

    /// @brief Remove one element from the end of the StackVector.
    /// @throws std::underflow_error if the StackVector is empty.
    void pop_back()
    {
        if (empty())
        {
            throw std::underflow_error("StackVector is empty");
        }
        --size_;
    }

    /// @brief Insert an element at a specified position.
    /// @param pos Position of the StackVector where the new elements are inserted provided as a random access iterator.
    /// @param value Value to be copied to the inserted elements.
    /// @return Random access iterator that points to elements.
    /// @throws std::overflow_error if the StackVector is full.
    /// @throws std::out_of_range error if the provided insert position is out of range.
    iterator insert(iterator pos, const T &value)
    {
        if (size_ >= N)
        {
            throw std::overflow_error("StackVector is full");
        }
        if (pos < begin() || pos > end())
        {
            throw std::out_of_range("Insert position out of range");
        }

        std::move_backward(pos, end(), end() + 1);
        *pos = value;
        ++size_;

        return pos;
    }

    /// @brief Insert an element at a specified position.
    /// @param pos Position of the StackVector where the new elements are inserted provided as a random access iterator.
    /// @param value Value to be moved to the inserted elements.
    /// @return Random access iterator that points to elements.
    /// @throws std::overflow_error if the StackVector is full.
    /// @throws std::out_of_range error if the provided insert position is out of range.
    iterator insert(iterator pos, T &&value)
    {
        if (size_ >= N)
        {
            throw std::overflow_error("StackVector is full");
        }
        if (pos < begin() || pos > end())
        {
            throw std::out_of_range("Insert position out of range");
        }

        std::move_backward(pos, end(), end() + 1);
        *pos = std::move(value);
        ++size_;

        return pos;
    }

    /// @brief Removes from the StackVector a single element.
    /// @param pos Random access iterator position corresponding to the element to be removed.
    /// @return New access iterator with a removed element.
    /// @throws std::out_of_range if invalid position is provided.
    iterator erase(iterator pos)
    {
        if (pos < begin() || pos >= end())
        {
            throw std::out_of_range("Invalid iterator position");
        }

        iterator next = pos + 1;
        std::move(next, end(), pos);
        --size_;

        return pos;
    }

    /// @brief Removes from the StackVector a range of elements [first, last).
    /// @param first Random access iterator type to the first element to be removed.
    /// @param last Random access iterator type to last non-inclusive element.
    /// @return New access iterator with removed elements.
    /// @throws std::out_of_range if invalid position is provided.
    iterator erase(iterator first, iterator last)
    {
        if (first < begin() || first > last || last > end())
        {
            throw std::out_of_range("Invalid iterator range");
        }

        iterator new_end = std::move(last, end(), first);
        size_ -= std::distance(first, last);

        return first;
    }

    /// @brief Get a reference to the element stored at the specified position.
    /// @param index Index to the element stored in the StackVector.
    /// @return Non-const reference to the element in the data.
    reference operator[](size_type index) noexcept
    {
        return data_[index];
    }

    /// @brief Get a constant reference to the element stored at the specified position.
    /// @param index Index to the element stored in the StackVector.
    /// @return Const reference to the element in the data.
    const_reference operator[](size_type index) const noexcept
    {
        return data_[index];
    }

    /// @brief Get a reference to the element stored at the specified position.
    /// @param index Index to the element stored in the StackVector.
    /// @return Non-const reference to the element in the data.
    /// @throws std::out_of_range If the index is out of range.
    reference at(size_type index) const
    {
        if (index >= size_)
        {
            throw std::out_of_range("StackVector index out of range");
        }
        return data_[index];
    }

    /// @brief Returns an iterator pointing to the first element in the StackVector.
    /// @return Non-const pointer to the first position of the data stored within the StackVector.
    iterator begin() noexcept
    {
        return data_.data();
    }

    /// @brief Returns an iterator pointing to the first element in the StackVector.
    /// @return Const pointer to the first position of the data stored within the StackVector.
    const_iterator cbegin() const noexcept
    {
        return data_.data();
    }

    /// @brief Returns an iterator referring to the element one past the end position of the StackVector.
    /// @return Non-const pointer to the past-the-end position of the data stored within the StackVector.
    iterator end() noexcept
    {
        return data_.data() + size_;
    }

    /// @brief Returns an iterator referring to the element one past the end position of the StackVector.
    /// @return Const pointer to the past-the-end position of the data stored within the StackVector.
    const_iterator cend() const noexcept
    {
        return data_.data() + size_;
    }

    /// @brief Returns a reverse iterator pointing to the last element in the StackVector.
    /// @return Non-const pointer to the last element of the data stored within the StackVector.
    reverse_iterator rbegin() noexcept
    {
        return reverse_iterator(end());
    }

    /// @brief Returns a const reverse iterator pointing to the last element in the StackVector.
    /// @return Const pointer to the last element of the data stored within the StackVector.
    const_reverse_iterator crbegin() const noexcept
    {
        return const_reverse_iterator(end());
    }

    /// @brief Returns a reverse iterator pointing to the before the start element in the StackVector.
    /// @return Non-const pointer to the element of the data stored within the StackVector preceding the first element.
    reverse_iterator rend() noexcept
    {
        return reverse_iterator(begin());
    }

    /// @brief Returns a const reverse iterator pointing to the before the start element in the StackVector.
    /// @return Const pointer to the element of the data stored within the StackVector preceding the first element.
    const_reverse_iterator crend() const noexcept
    {
        return const_reverse_iterator(begin());
    }

  private:
    std::array<T, N> data_;
    size_type size_;
};
} // namespace lidar_processing::utilities

#endif // STACK_VECTOR_HPP