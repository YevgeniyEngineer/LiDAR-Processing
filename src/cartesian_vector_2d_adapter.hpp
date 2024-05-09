#ifndef LIDAR_PROCESSING__CARTESIAN_VECTOR_2D_ADAPTER_HPP
#define LIDAR_PROCESSING__CARTESIAN_VECTOR_2D_ADAPTER_HPP

#include <cassert>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <vector>

namespace containers
{
class CartesianVector2DAdapter final
{
  public:
    class Point2D final
    {
      public:
        void x(float x) noexcept;
        void y(float y) noexcept;

        float &x() noexcept;
        float &y() noexcept;

        float x() const noexcept;
        float y() const noexcept;

        void set(float x, float y) noexcept;

      private:
        friend class CartesianVector2DAdapter;
        friend class KDTree2D;

        explicit Point2D(float *ptr) noexcept;

        float *ptr_;
    };

    CartesianVector2DAdapter(const std::shared_ptr<std::vector<float>> &data);
    ~CartesianVector2DAdapter() noexcept;

    Point2D operator[](std::size_t index) noexcept;
    const Point2D operator[](std::size_t index) const;

    Point2D at(std::size_t index);
    const Point2D at(std::size_t index) const;

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
    void emplace_back(float x, float y);
    void assign(std::size_t new_point_count, float x, float y);
    void pop_back() noexcept;

  private:
    // Stores coordinates: x1, y1, z1, x2, y2, z2, x3, y3, z3, ... , xN, yN, zN
    std::shared_ptr<std::vector<float>> data_;
};

inline void CartesianVector2DAdapter::Point2D::x(float x) noexcept
{
    ptr_[0] = x;
}

inline void CartesianVector2DAdapter::Point2D::y(float y) noexcept
{
    ptr_[1] = y;
}

inline float &CartesianVector2DAdapter::Point2D::x() noexcept
{
    return ptr_[0];
}

inline float &CartesianVector2DAdapter::Point2D::y() noexcept
{
    return ptr_[1];
}

inline float CartesianVector2DAdapter::Point2D::x() const noexcept
{
    return ptr_[0];
}

inline float CartesianVector2DAdapter::Point2D::y() const noexcept
{
    return ptr_[1];
}

inline void CartesianVector2DAdapter::Point2D::set(float x, float y) noexcept
{
    ptr_[0] = x;
    ptr_[1] = y;
}

inline CartesianVector2DAdapter::Point2D::Point2D(float *ptr) noexcept : ptr_(ptr)
{
}

inline CartesianVector2DAdapter::CartesianVector2DAdapter(const std::shared_ptr<std::vector<float>> &data) : data_(data)
{
    if (!data_)
    {
        throw std::invalid_argument("Data vector cannot be null");
    }
}

inline CartesianVector2DAdapter::~CartesianVector2DAdapter() noexcept
{
    data_.reset();
}

inline CartesianVector2DAdapter::Point2D CartesianVector2DAdapter::operator[](std::size_t index) noexcept
{
    assert(index < size() && "Index out of bounds");
    return Point2D(&(*data_)[index * 2]);
}

inline const CartesianVector2DAdapter::Point2D CartesianVector2DAdapter::operator[](std::size_t index) const
{
    assert(index < size() && "Index out of bounds");
    return Point2D(&(*data_)[index * 2]);
}

inline CartesianVector2DAdapter::Point2D CartesianVector2DAdapter::at(std::size_t index)
{
    if (index >= size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return Point2D(&(*data_)[index * 2]);
}

inline const CartesianVector2DAdapter::Point2D CartesianVector2DAdapter::at(std::size_t index) const
{
    if (index >= size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return Point2D(&(*data_)[index * 2]);
}

inline float &CartesianVector2DAdapter::operator()(std::size_t index, std::size_t dim) noexcept
{
    assert((dim < 2) && "Dimension out of bounds");
    assert((index * 2 + dim < data_->size()) && "Index out of bounds");
    return data_->operator[](index * 2 + dim);
}

inline float CartesianVector2DAdapter::operator()(std::size_t index, std::size_t dim) const noexcept
{
    assert((dim < 2) && "Dimension out of bounds");
    assert((index * 2 + dim < data_->size()) && "Index out of bounds");
    return data_->operator[](index * 2 + dim);
}

inline float &CartesianVector2DAdapter::at(std::size_t index, std::size_t dim)
{
    if (dim > 1)
    {
        throw std::out_of_range("Dimension out of bounds");
    }
    if (index * 2 + dim >= data_->size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data_->operator[](index * 3 + dim);
}

inline float CartesianVector2DAdapter::at(std::size_t index, std::size_t dim) const
{
    if (dim > 1)
    {
        throw std::out_of_range("Dimension out of bounds");
    }
    if (index * 2 + dim >= data_->size())
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data_->operator[](index * 3 + dim);
}

inline void CartesianVector2DAdapter::clear() noexcept
{
    return data_->clear();
}

inline bool CartesianVector2DAdapter::empty() const noexcept
{
    return data_->empty();
}

inline std::size_t CartesianVector2DAdapter::size() const noexcept
{
    return data_->size() / 2;
}

inline std::size_t CartesianVector2DAdapter::capacity() const noexcept
{
    return data_->capacity() / 2;
}

inline void CartesianVector2DAdapter::resize(std::size_t new_point_count)
{
    data_->resize(new_point_count * 2);
}

inline void CartesianVector2DAdapter::reserve(std::size_t point_count)
{
    data_->reserve(point_count * 2);
}

inline void CartesianVector2DAdapter::emplace_back(float x, float y)
{
    data_->push_back(x);
    data_->push_back(y);
}

inline void CartesianVector2DAdapter::assign(std::size_t new_point_count, float x, float y)
{
    data_->resize(new_point_count * 2);

    for (std::size_t i = 0; i < data_->size(); i += 2)
    {
        (*data_)[i] = x;
        (*data_)[i + 1] = y;
    }
}

inline void CartesianVector2DAdapter::pop_back() noexcept
{
    if (empty())
    {
        return;
    }
    data_->resize(data_->size() - 2);
}
} // namespace containers

#endif // LIDAR_PROCESSING__CARTESIAN_VECTOR_2D_ADAPTER_HPP
