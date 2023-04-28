#ifndef CONVEX_HULL_HPP
#define CONVEX_HULL_HPP

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <type_traits>
#include <variant>
#include <vector>

namespace geom
{
/// @brief Class that stores orientation formed by three 2D points
enum class Orientation
{
    COUNTERCLOCKWISE,
    CLOCKWISE,
    COLLINEAR
};

/// @brief Class that stores available Convex Hull algorithms for selection
enum class ConvexHullAlgorithm
{
    GRAHAM_SCAN,
    ANDREW_MONOTONE_CHAIN,
    JARVIS_MARCH,
    CHAN
};

/// @brief Point type used in calculation of 2D Convex Hull
template <typename T> struct Point
{
    static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value,
                  "Point can only be templated with integer or floating-point types");

    T x, y;

    constexpr Point(T x, T y) : x(x), y(y)
    {
    }

    bool operator<(const Point &other) const noexcept
    {
        if constexpr (std::is_integral<T>::value)
        {
            return (y < other.y) || ((y == other.y) && (x < other.x));
        }
        else
        {
            return (y < other.y) || (std::fabs(y - other.y) < std::numeric_limits<T>::epsilon() && (x < other.x));
        }
    }

    bool operator==(const Point &other) const noexcept
    {
        if constexpr (std::is_integral<T>::value)
        {
            return (x == other.x) && (y == other.y);
        }
        else
        {
            return (std::fabs(x - other.x) < std::numeric_limits<T>::epsilon()) &&
                   (std::fabs(y - other.y) < std::numeric_limits<T>::epsilon());
        }
    }
};

/// @brief Function to return the cross product of two vectors (p1, p2) and (p1, p3)
template <typename T> inline T crossProduct(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3) noexcept
{
    const auto x1 = p2.x - p1.x;
    const auto y1 = p2.y - p1.y;
    const auto x2 = p3.x - p1.x;
    const auto y2 = p3.y - p1.y;
    return x1 * y2 - x2 * y1;
}

/// @brief Helper function to compute the square of the Euclidean distance between two points
template <typename T> inline T squaredDistance(const Point<T> &p1, const Point<T> &p2) noexcept
{
    const auto dx = p2.x - p1.x;
    const auto dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}

/// @brief Comparator function to sort points by polar angle with respect to the reference point
template <typename T>
inline bool comparePolarAngle(const Point<T> &ref, const Point<T> &p1, const Point<T> &p2) noexcept
{
    const auto cross_product = crossProduct(ref, p1, p2);
    if (cross_product == 0)
    {
        return squaredDistance(ref, p1) < squaredDistance(ref, p2);
    }
    return (cross_product > 0);
}

/// @brief Function to check if three points form a clockwise or counterclockwise order
template <typename T>
inline Orientation getOrientation(const Point<T> &p1, const Point<T> &p2, const Point<T> &p3) noexcept
{
    const auto cross_product = crossProduct(p1, p2, p3);

    if (cross_product > 0)
    {
        return Orientation::COUNTERCLOCKWISE;
    }
    else if (cross_product < 0)
    {
        return Orientation::CLOCKWISE;
    }
    else
    {
        return Orientation::COLLINEAR;
    }
}

/// @brief Return convex hull indices calculated using Graham-Andrew algorithm
template <typename T>
std::vector<int> constructGrahamScanConvexHull(const std::vector<Point<T>> &points, Orientation orientation)
{
    int n = static_cast<int>(points.size());
    if (n < 3)
    {
        // Convex Hull is not possible for less than 3 points
        return {};
    }

    // Find point with the lowest y-coordinate (and lowest x coordinate if there is a tie)
    int min_index = 0;
    for (int i = 1; i < n; ++i)
    {
        if (points[i] < points[min_index])
        {
            min_index = i;
        }
    }

    // Swap the first point and the point with the lowest y-coordinate
    std::vector<Point<T>> sorted_points = points;
    std::swap(sorted_points[0], sorted_points[min_index]);

    // Sort the remaining points based on their polar angle with respect to the reference point
    const Point<T> &ref_point = sorted_points[0];
    std::sort(
        sorted_points.begin() + 1, sorted_points.end(),
        [&](const Point<T> &p1, const Point<T> &p2) noexcept -> bool { return comparePolarAngle(ref_point, p1, p2); });

    // Initialize the convex hull with the first three sorted points
    std::vector<int> hull_indices = {0, 1, 2};
    int hull_size = 3;

    // Process remaining points
    for (int i = 3; i < n; ++i)
    {
        // Remove the last point from the hull while it makes a clockwise turn with the next point
        while (hull_size >= 2 &&
               getOrientation(sorted_points[hull_indices[hull_size - 2]], sorted_points[hull_indices[hull_size - 1]],
                              sorted_points[i]) != Orientation::COUNTERCLOCKWISE)
        {
            hull_indices.pop_back();
            --hull_size;
        }

        // Add the next point index to the hull
        hull_indices.push_back(i);
        ++hull_size;
    }

    // Convert the indices of the sorted_points vector back to the original points vector
    std::vector<int> original_indices(hull_size);
    for (int i = 0; i < hull_size; ++i)
    {
        const auto hull_index = hull_indices[i];
        for (int j = 0; j < n; ++j)
        {
            if (sorted_points[hull_index] == points[j])
            {
                original_indices[i] = j;
                break;
            }
        }
    }

    // Check orientation, and reverse order if orientation set to CLOCKWISE
    if (orientation == Orientation::CLOCKWISE)
    {
        std::reverse(original_indices.begin(), original_indices.end());
    }
    return original_indices;
}

/// @brief Andrew's Monotone Chain convex hull algorithm
template <typename T>
std::vector<int> constructAndrewMonotoneChainConvexHull(const std::vector<Point<T>> &points, Orientation orientation)
{
    int n = static_cast<int>(points.size());
    if (n < 3)
    {
        // Convex Hull is not possible for less than 3 points
        return {};
    }

    // Preallocate indices and sort points
    std::vector<int> hull_indices(2 * n);
    std::vector<Point<T>> sorted_points = points;
    std::sort(sorted_points.begin(), sorted_points.end());

    // Compute lower hull
    int k = 0;
    for (int i = 0; i < n; ++i)
    {
        while (k >= 2 && getOrientation(sorted_points[hull_indices[k - 2]], sorted_points[hull_indices[k - 1]],
                                        sorted_points[i]) != Orientation::COUNTERCLOCKWISE)
        {
            --k;
        }
        hull_indices[k++] = i;
    }

    // Compute upper hull
    for (int i = n - 2, t = k + 1; i >= 0; --i)
    {
        while (k >= t && getOrientation(sorted_points[hull_indices[k - 2]], sorted_points[hull_indices[k - 1]],
                                        sorted_points[i]) != Orientation::COUNTERCLOCKWISE)
        {
            --k;
        }
        hull_indices[k++] = i;
    }

    // Resize convex hull to contain only required points
    hull_indices.resize(k - 1);

    // Convert the indices of the sorted_points vector back to the original points vector
    for (int i = 0; i < k - 1; ++i)
    {
        const auto hull_index = hull_indices[i];
        for (int j = 0; j < n; ++j)
        {
            if (sorted_points[hull_index] == points[j])
            {
                hull_indices[i] = j;
                break;
            }
        }
    }

    // Check orientation, and reverse order if orientation set to CLOCKWISE
    if (orientation == Orientation::CLOCKWISE)
    {
        std::reverse(hull_indices.begin(), hull_indices.end());
    }
    return hull_indices;
}

/// @brief Construct Convex Hull using Jarvis March algorithm
template <typename T>
std::vector<int> constructJarvisMarchConvexHull(const std::vector<Point<T>> &points, Orientation orientation)
{
    int n = static_cast<int>(points.size());
    if (n < 3)
    {
        // Convex Hull is not possible for less than 3 points
        return {};
    }

    // Find the leftmost point (with the lowest x-coordinate)
    int leftmost = 0;
    for (int i = 1; i < n; ++i)
    {
        if (points[i].x < points[leftmost].x)
        {
            leftmost = i;
        }
    }

    // Start from leftmost point, keep moving counterclockwise
    // until reaching the start point again.
    std::vector<int> hull_indices;
    int p = leftmost;
    int q;

    do
    {
        // Add current point to result
        hull_indices.push_back(p);

        // Initialize q as the next point in the list
        q = (p + 1) % n;

        // Iterate over all points to find the point with the smallest polar angle
        // with respect to the current point (p)
        for (int i = 0; i < n; ++i)
        {
            // If i is more counterclockwise than current q, then update q
            if (getOrientation(points[p], points[i], points[q]) == Orientation::COUNTERCLOCKWISE)
            {
                q = i;
            }
        }

        // Set p to be q for the next iteration
        p = q;

    } while (p != leftmost); // Continue until we reach the starting point again

    // Check orientation, and reverse order if orientation set to CLOCKWISE
    if (orientation == Orientation::CLOCKWISE)
    {
        std::reverse(hull_indices.begin(), hull_indices.end());
    }
    return hull_indices;
}

/// @brief Partitions vector into approximately n equal pieces
template <typename T>
std::pair<std::vector<std::vector<T>>, std::vector<std::vector<int>>> partitionVector(const std::vector<T> &vec,
                                                                                      int number_of_subsets)
{
    int size = vec.size();
    int subset_size = size / number_of_subsets;
    int remainder = size % number_of_subsets;

    std::vector<std::vector<T>> subsets(number_of_subsets);
    std::vector<std::vector<int>> subset_indices(number_of_subsets);
    int current = 0;
    for (int i = 0; i < number_of_subsets; ++i)
    {
        int current_subset_size = subset_size + (i < remainder ? 1 : 0);

        subsets[i].reserve(current_subset_size);
        subset_indices[i].reserve(current_subset_size);

        for (int j = 0; j < current_subset_size; ++j)
        {
            subsets[i].push_back(vec[current]);
            subset_indices[i].push_back(current);
            ++current;
        }
    }

    return std::make_pair(subsets, subset_indices);
}

/// @brief Construct Convex Hull using Chan's algorithm, based on Andrew's Monotone Chain and Jarvis March
template <typename T>
std::vector<int> constructChanConvexHull(const std::vector<Point<T>> &points, Orientation orientation)
{
    int n = static_cast<int>(points.size());
    if (n < 3)
    {
        // Convex Hull is not possible for less than 3 points
        return {};
    }

    // Find the number of subsets to split the points into
    const int number_of_subsets = static_cast<int>(std::ceil(std::sqrt(n)));
    const int number_of_points_within_subset = n / number_of_subsets;

    // Divide points into number_of_subsets subsets each containing number_of_points_within_subset points
    const auto [subsets, subset_indices] = partitionVector(points, number_of_subsets);

    // Compute convex hull using Graham's scan for each subset
    std::vector<std::vector<int>> convex_hulls_indices(subsets.size());
    for (int subset_no = 0; subset_no < subsets.size(); ++subset_no)
    {
        // Construct convex hull for current subset
        convex_hulls_indices[subset_no] = constructAndrewMonotoneChainConvexHull(subsets[subset_no], orientation);
    }

    // Merge convex hull points
    std::vector<Point<T>> merged_points;
    merged_points.reserve(convex_hulls_indices.size() * number_of_points_within_subset);

    std::vector<int> merged_indices;
    merged_indices.reserve(convex_hulls_indices.size() * number_of_points_within_subset);

    for (int subset_no = 0; subset_no < subsets.size(); ++subset_no)
    {
        const auto &indices = convex_hulls_indices[subset_no];
        const auto &original_indices = subset_indices[subset_no];
        for (const auto idx : indices)
        {
            const auto original_index = original_indices[idx];
            merged_points.push_back(points[original_index]);
            merged_indices.push_back(original_index);
        }
    }

    // Merge convex hulls using Jarvis March and obtain the indices of the points in the hull
    std::vector<int> hull_indices = constructJarvisMarchConvexHull(merged_points, orientation);

    // Convert the indices of the merged_points vector back to the original points vector
    std::vector<int> original_indices(hull_indices.size());
    for (int i = 0; i < hull_indices.size(); ++i)
    {
        original_indices[i] = merged_indices[hull_indices[i]];
    }
    return original_indices;
}

/// @brief Main method that calls relevant functions based on provided inputs
template <typename T>
std::vector<int> constructConvexHull(const std::vector<Point<T>> &points,
                                     ConvexHullAlgorithm algorithm = ConvexHullAlgorithm::GRAHAM_SCAN,
                                     Orientation orientation = Orientation::COUNTERCLOCKWISE)
{
    if (orientation == Orientation::COLLINEAR)
    {
        std::cerr << "Orientation::COLLINEAR only supported internally. Returning empty hull." << std::endl;
        return {};
    }

    switch (algorithm)
    {
    case ConvexHullAlgorithm::GRAHAM_SCAN: {
        return constructGrahamScanConvexHull(points, orientation);
    }
    case ConvexHullAlgorithm::ANDREW_MONOTONE_CHAIN: {
        return constructAndrewMonotoneChainConvexHull(points, orientation);
    }
    case ConvexHullAlgorithm::JARVIS_MARCH: {
        return constructJarvisMarchConvexHull(points, orientation);
    }
    case ConvexHullAlgorithm::CHAN: {
        return constructChanConvexHull(points, orientation);
    }
    default: {
        return {};
    }
    }
}

} // namespace geom

#endif // CONVEX_HULL_HPP