#ifndef GEOMETRIC_OPERATIONS
#define GEOMETRIC_OPERATIONS

#include <algorithm> // std::sort
#include <execution> // std::execution
#include <iterator>  // std::back_insert_iterator
#include <vector>    // std::vector

namespace lidar_processing
{
/// @brief Checks if points oriented counterclockwise
/// @tparam PointT 2D point containing x, y coordinates
/// @param point_1 First point
/// @param point_2 Second point
/// @param point_3 Third point
/// @return True if clockwise, else False
template <typename PointT>
inline static bool counterclockwise(const PointT &point_1, const PointT &point_2, const PointT &point_3) noexcept
{
    return (point_2.x - point_1.x) * (point_3.y - point_1.y) - (point_2.y - point_1.y) * (point_3.x - point_1.x) > 0.0;
}

/// @brief Graham - Andrew algorithm
/// @tparam PointT 2D point containing x, y coordinates
/// @param points Vector of points
/// @param hull Convex hull formed from 2D points
template <typename PointT>
inline static void constructGrahamAndrewConvexHull(std::vector<PointT> points, std::vector<PointT> &hull) noexcept
{
    // Clean old point cache
    hull.clear();

    // If number of points less than 3, no need to compute hull
    int n = points.size();
    if (n <= 3)
    {
        hull = std::move(points);
        return;
    }

    // Sort points lexicographically
    if (n > 500)
    {
        std::sort(std::execution::par, points.begin(), points.end());
    }
    else
    {
        std::sort(std::execution::unseq, points.begin(), points.end());
    }

    // Reserve points for convex hull
    hull.resize(n * 2);

    // Compute lower hull
    int k = 0;
    for (int i = 0; i < n; hull[k++] = points[i++])
    {
        for (; (k > 1) && counterclockwise(hull[k - 2], hull[k - 1], points[i]); --k)
        {
            ;
        }
    }

    // Compute upper hull
    for (int i = n - 2, t = k; i >= 0; hull[k++] = points[i--])
    {
        for (; (k > t) && counterclockwise(hull[k - 2], hull[k - 1], points[i]); --k)
        {
            ;
        }
    }

    // Resize convex hull to contain only required points
    hull.resize(k - 1 - (hull[0] == hull[1]));
}
} // namespace lidar_processing

#endif // GEOMETRIC_OPERATIONS