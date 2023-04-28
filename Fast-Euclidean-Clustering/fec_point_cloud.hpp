#ifndef FEC_POINT_CLOUD_HPP
#define FEC_POINT_CLOUD_HPP

#include <array>       // std::array
#include <cstdint>     // std::size_t
#include <stdexcept>   // std::runtime_error
#include <type_traits> // std::enable_if_t
#include <vector>      // std::vector

namespace clustering
{
/// @brief Definition of the point struct
template <typename CoordinateType, std::size_t number_of_dimensions,
          typename = std::enable_if_t<(number_of_dimensions == 2) || (number_of_dimensions == 3)>>
using FECPoint = std::array<CoordinateType, number_of_dimensions>;

/// @brief Point Struct defined for 3 dimensions
template <typename CoordinateType, std::size_t number_of_dimensions> struct FECPointCloud final
{
    // Container for points
    using PointType = FECPoint<CoordinateType, number_of_dimensions>;
    std::vector<PointType> points;

    FECPointCloud(const FECPointCloud &) = default;
    FECPointCloud &operator=(const FECPointCloud &) = default;
    FECPointCloud(FECPointCloud &&) noexcept = default;
    FECPointCloud &operator=(FECPointCloud &&) noexcept = default;
    FECPointCloud() = default;
    FECPointCloud(const std::vector<PointType> &points) : points(points){};
    FECPointCloud(std::vector<PointType> &&points) : points(std::move(points)){};
    ~FECPointCloud() = default;

    /// @brief Return the number of points in the cloud
    inline std::size_t kdtree_get_point_count() const noexcept
    {
        return points.size();
    }

    /// @brief Get a point along the specified dimension
    inline CoordinateType kdtree_get_pt(const std::size_t idx, const std::size_t dim) const noexcept
    {
        // Assuming dim is always correct
        return points[idx][dim];
    }

    /// @brief Optional bounding box computation
    template <class Bbox> inline bool kdtree_get_bbox(Bbox & /* bb */) const noexcept
    {
        return false;
    }

    /// @brief Returns number of points
    inline std::size_t size() const noexcept
    {
        return points.size();
    }

    /// @brief Adds a new point to the container
    inline void push_back(const PointType &point)
    {
        points.push_back(point);
    }

    /// @brief Adds a new point to the container
    inline void push_back(PointType &&point)
    {
        points.push_back(point);
    }

    /// @brief Reserve memory for the container
    inline void reserve(const std::size_t size)
    {
        points.reserve(size);
    }

    /// @brief Resize the container
    inline void resize(const std::size_t size)
    {
        points.resize(size);
    }

    /// @brief Get non-constant reference to the point at specified index
    inline PointType &operator[](const std::size_t index) noexcept
    {
        return points[index];
    }

    /// @brief Get constant reference to the point at specified index
    inline const PointType &operator[](const std::size_t index) const noexcept
    {
        return points[index];
    }
};
} // namespace clustering

#endif // FEC_POINT_CLOUD_HPP