#ifndef GEOMETRIC_OPERATIONS
#define GEOMETRIC_OPERATIONS

#include <algorithm> // std::sort
#include <execution> // std::execution
#include <iterator>  // std::back_insert_iterator
#include <utility>   // std::move
#include <vector>    // std::vector

namespace lidar_processing
{
/// @brief Checks if points oriented counterclockwise
/// @tparam PointT 2D point containing x, y coordinates
/// @param point_1 First point
/// @param point_2 Second point
/// @param point_3 Third point
/// @return True if counterclockwise, else False (counterclockwise = to the left, Clockwise = to the right)
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
        std::move(std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()),
                  std::back_inserter(hull));

        points.erase(points.begin(), points.end());
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

// Gift wrapping algorithm to compute the convex hull of a set of points
template <typename PointT>
inline static void constructJarvisMarchConvexHull(const std::vector<PointT> &points, std::vector<PointT> &hull) noexcept
{
    // Clean old point cache
    hull.clear();

    // If number of points less than 3, no need to compute hull
    int n = points.size();
    if (n <= 3)
    {
        std::copy(points.begin(), points.end(), std::back_inserter(hull));
        return;
    }

    // Find the leftmost point
    int leftmost = 0;
    for (int i = 1; i < n; ++i)
    {
        if (points[i].x < points[leftmost].x)
        {
            leftmost = i;
        }
    }

    // Start from leftmost point, keep moving counterclockwise
    // until reaching the start point again.  This loop runs O(h)
    // times where h is number of points in result or output.
    int p = leftmost;
    int q;
    do
    {
        // Add current point to result
        hull.emplace_back(points[p]);

        // Search for a point 'q' such that orientation(p, x,
        // q) is counterclockwise for all points 'x'. The idea
        // is to keep track of last visited most counterclock-
        // wise point in q. If any point 'i' is more counterclock-
        // wise than q, then update q.
        q = (p + 1) % n;
        for (int i = 0; i < n; ++i)
        {
            // If i is more counterclockwise than current q, then
            // update q
            if (counterclockwise(points[p], points[i], points[q]))
            {
                q = i;
            }
        }
        // Now q is the most counterclockwise with respect to p
        // Set p as q for next iteration, so that q is added to
        // result 'hull'
        p = q;
    } while (p != leftmost); // While we don't come to first point
}

template <typename PointT>
inline static void constructChanConvexHull(std::vector<PointT> points, std::vector<PointT> &hull) noexcept
{
    // Clean old point cache
    hull.clear();

    // If number of points less than 3, no need to compute hull
    int n = points.size();
    if (n <= 3)
    {
        std::move(std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()),
                  std::back_inserter(hull));

        points.erase(points.begin(), points.end());
        return;
    }

    // Find the number of subsets to split the points into
    int number_of_subsets = std::ceil(std::sqrt(n));
    int number_of_points_within_subset = points.size() / number_of_subsets;

    // Divide points into number_of_subsets subsets each containing number_of_points_within_subset points
    std::vector<std::vector<PointT>> subsets(number_of_subsets);
    for (int subset_no = 0; subset_no < number_of_subsets; ++subset_no)
    {
        // Get the range of the next set of m elements
        auto start_it = std::next(points.begin(), subset_no * number_of_points_within_subset);
        auto stop_it =
            std::next(points.begin(), subset_no * number_of_points_within_subset + number_of_points_within_subset);

        // Allocate memory of the sub-vector
        subsets[subset_no].reserve(number_of_points_within_subset);

        // Code to handle the last sub-vector if it contains less elements than other sub-vectors
        if (subset_no * number_of_points_within_subset + number_of_points_within_subset > points.size())
        {
            stop_it = points.end();
        }

        // Copy elements from the input range to the sub-vector
        std::move(std::make_move_iterator(start_it), std::make_move_iterator(stop_it),
                  std::back_inserter(subsets[subset_no]));
    }

    // After moving elements from points vector, all elements are not in indeterminate state
    points.erase(points.begin(), points.end());

    // Compute convex hull using Graham's scan for each subset
    std::vector<std::vector<PointT>> convex_hulls(subsets.size());
    std::vector<int> subset_numbers(subsets.size());
    std::iota(subset_numbers.begin(), subset_numbers.end(), 0);
    std::for_each(std::execution::par, subset_numbers.cbegin(), subset_numbers.cend(),
                  [&convex_hulls, &subsets](const int subset_no) {
                      // Construct convex hull for current subset
                      // This will fill convex_hulls[subset_no] with the hull corresponding to subsets[subset_no]
                      constructGrahamAndrewConvexHull(subsets[subset_no], convex_hulls[subset_no]);
                  });

    // Merge convex hull points
    std::vector<PointT> merged_points;
    for (const auto &hull_points : convex_hulls)
    {
        merged_points.reserve(merged_points.size() + hull_points.size());
        std::move(std::make_move_iterator(hull_points.begin()), std::make_move_iterator(hull_points.end()),
                  std::back_inserter(merged_points));
    }

    // After moving elements from convex_hulls into merged_points, it is in indeterminate state
    convex_hulls.erase(convex_hulls.begin(), convex_hulls.end());

    // Merge convex hulls using Jarvis March
    constructJarvisMarchConvexHull(merged_points, hull);
}
} // namespace lidar_processing

#endif // GEOMETRIC_OPERATIONS