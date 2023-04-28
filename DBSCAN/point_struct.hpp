#ifndef POINT_STRUCT_HPP
#define POINT_STRUCT_HPP

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
using Point = std::array<CoordinateType, number_of_dimensions>;

/// @brief Point Struct defined for 3 dimensions
template <typename CoordinateType, std::size_t number_of_dimensions> struct PointCloud final
{
    // Container for points
    std::vector<Point<CoordinateType, number_of_dimensions>> points;

    /// @brief Return the number of points in the cloud
    inline std::size_t kdtree_get_point_count() const noexcept
    {
        return points.size();
    }

    /// @brief Get a point along the specified dimension
    inline CoordinateType kdtree_get_pt(const std::size_t idx, const std::size_t dim) const
    {
        // Assuming dim is always correct
        return points[idx][dim];
    }

    /// @brief Optional bounding box computation
    template <class Bbox> inline bool kdtree_get_bbox(Bbox & /* bb */) const
    {
        return false;
    }
};
} // namespace clustering

#endif // POINT_STRUCT_HPP