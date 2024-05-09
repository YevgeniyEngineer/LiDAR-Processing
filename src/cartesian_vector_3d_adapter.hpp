/*
 * Copyright (c) 2024 Yevgeniy Simonov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef CONTAINERS__CARTESIAN_VECTOR_3D_ADAPTER_HPP
#define CONTAINERS__CARTESIAN_VECTOR_3D_ADAPTER_HPP

#include <cassert>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <vector>

namespace containers
{
class CartesianVector3DAdapter final
{
  public:
    class Point3D final
    {
      public:
        void x(float x) noexcept;
        void y(float y) noexcept;
        void z(float z) noexcept;

        float &x() noexcept;
        float &y() noexcept;
        float &z() noexcept;

        float x() const noexcept;
        float y() const noexcept;
        float z() const noexcept;

        void set(float x, float y, float z) noexcept;

      private:
        friend class CartesianVector3DAdapter;
        friend class KDTree3D;

        explicit Point3D(float *ptr) noexcept;

        float *ptr_;
    };

    CartesianVector3DAdapter(const std::shared_ptr<std::vector<float>> &data);
    ~CartesianVector3DAdapter() noexcept;

    Point3D operator[](std::size_t index) noexcept;
    const Point3D operator[](std::size_t index) const;

    Point3D at(std::size_t index);
    const Point3D at(std::size_t index) const;

    float &operator()(std::size_t index, std::size_t dim) noexcept;
    float operator()(std::size_t index, std::size_t dim) const noexcept;

    float &at(std::size_t index, std::size_t dim);
    float at(std::size_t index, std::size_t dim) const;

    void clear() noexcept;
    bool empty() const noexcept;
    std::size_t size() const noexcept;
    std::size_t capacity() const noexcept;
    void resize(std::size_t new_point_count);
    void reserve(std::size_t point_count);
    void emplace_back(float x, float y, float z);
    void assign(std::size_t new_point_count, float x, float y, float z);
    void pop_back() noexcept;

  private:
    // Stores coordinates: x1, y1, z1, x2, y2, z2, x3, y3, z3, ... , xN, yN, zN
    std::shared_ptr<std::vector<float>> data_;
};

inline void CartesianVector3DAdapter::Point3D::x(float x) noexcept
{
    ptr_[0] = x;
}

inline void CartesianVector3DAdapter::Point3D::y(float y) noexcept
{
    ptr_[1] = y;
}

inline void CartesianVector3DAdapter::Point3D::z(float z) noexcept
{
    ptr_[2] = z;
}

inline float &CartesianVector3DAdapter::Point3D::x() noexcept
{
    return ptr_[0];
}

inline float &CartesianVector3DAdapter::Point3D::y() noexcept
{
    return ptr_[1];
}

inline float &CartesianVector3DAdapter::Point3D::z() noexcept
{
    return ptr_[2];
}

inline float CartesianVector3DAdapter::Point3D::x() const noexcept
{
    return ptr_[0];
}

inline float CartesianVector3DAdapter::Point3D::y() const noexcept
{
    return ptr_[1];
}

inline float CartesianVector3DAdapter::Point3D::z() const noexcept
{
    return ptr_[2];
}

inline void CartesianVector3DAdapter::Point3D::set(float x, float y, float z) noexcept
{
    ptr_[0] = x;
    ptr_[1] = y;
    ptr_[2] = z;
}

inline CartesianVector3DAdapter::Point3D::Point3D(float *ptr) noexcept : ptr_(ptr)
{
}

inline CartesianVector3DAdapter::CartesianVector3DAdapter(const std::shared_ptr<std::vector<float>> &data) : data_(data)
{
    if (!data_)
    {
        throw std::invalid_argument("Data vector cannot be null");
    }
}

inline CartesianVector3DAdapter::~CartesianVector3DAdapter() noexcept
{
    data_.reset();
}

inline CartesianVector3DAdapter::Point3D CartesianVector3DAdapter::operator[](std::size_t index) noexcept
{
    assert(index < size() && "Index out of bounds");
    return Point3D(&(*data_)[index * 3]);
}

inline const CartesianVector3DAdapter::Point3D CartesianVector3DAdapter::operator[](std::size_t index) const
{
    assert(index < size() && "Index out of bounds");
    return Point3D(&(*data_)[index * 3]);
}

inline CartesianVector3DAdapter::Point3D CartesianVector3DAdapter::at(std::size_t index)
{
    if (index >= size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return Point3D(&(*data_)[index * 3]);
}

inline const CartesianVector3DAdapter::Point3D CartesianVector3DAdapter::at(std::size_t index) const
{
    if (index >= size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return Point3D(&(*data_)[index * 3]);
}

inline float &CartesianVector3DAdapter::operator()(std::size_t index, std::size_t dim) noexcept
{
    assert((dim < 3) && "Dimension out of bounds");
    assert((index * 3 + dim < data_->size()) && "Index out of bounds");
    return (*data_)[index * 3 + dim];
}

inline float CartesianVector3DAdapter::operator()(std::size_t index, std::size_t dim) const noexcept
{
    assert((dim < 3) && "Dimension out of bounds");
    assert((index * 3 + dim < data_->size()) && "Index out of bounds");
    return (*data_)[index * 3 + dim];
}

inline float &CartesianVector3DAdapter::at(std::size_t index, std::size_t dim)
{
    if (dim > 2)
    {
        throw std::out_of_range("Dimension out of bounds");
    }
    if (index * 3 + dim >= data_->size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return (*data_)[index * 3 + dim];
}

inline float CartesianVector3DAdapter::at(std::size_t index, std::size_t dim) const
{
    if (dim > 2)
    {
        throw std::out_of_range("Dimension out of bounds");
    }
    if (index * 3 + dim >= data_->size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return (*data_)[index * 3 + dim];
}

inline void CartesianVector3DAdapter::clear() noexcept
{
    return data_->clear();
}

inline bool CartesianVector3DAdapter::empty() const noexcept
{
    return data_->empty();
}

inline std::size_t CartesianVector3DAdapter::size() const noexcept
{
    return data_->size() / 3;
}

inline std::size_t CartesianVector3DAdapter::capacity() const noexcept
{
    return data_->capacity() / 3;
}

inline void CartesianVector3DAdapter::resize(std::size_t new_point_count)
{
    data_->resize(new_point_count * 3);
}

inline void CartesianVector3DAdapter::reserve(std::size_t point_count)
{
    data_->reserve(point_count * 3);
}

inline void CartesianVector3DAdapter::emplace_back(float x, float y, float z)
{
    data_->push_back(x);
    data_->push_back(y);
    data_->push_back(z);
}

inline void CartesianVector3DAdapter::assign(std::size_t new_point_count, float x, float y, float z)
{
    data_->resize(new_point_count * 3);

    for (std::size_t i = 0; i < data_->size(); i += 3)
    {
        (*data_)[i] = x;
        (*data_)[i + 1] = y;
        (*data_)[i + 2] = z;
    }
}

inline void CartesianVector3DAdapter::pop_back() noexcept
{
    if (empty())
    {
        return;
    }
    data_->resize(data_->size() - 3);
}
} // namespace containers

#endif // CONTAINERS__CARTESIAN_VECTOR_3D_ADAPTER_HPP
