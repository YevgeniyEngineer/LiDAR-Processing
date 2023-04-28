#ifndef KDTREE_HPP
#define KDTREE_HPP

#include <algorithm>   // std::sort, std::nth_element
#include <array>       // std::array
#include <cmath>       // std::floor
#include <cstdint>     // std::size_t
#include <execution>   // std::execution
#include <functional>  // std::ref
#include <future>      // std::async
#include <iterator>    // std::distance
#include <limits>      // std::numeric_limits
#include <numeric>     // std::iota
#include <queue>       // std::priority_queue
#include <stdexcept>   // std::runtime_error
#include <thread>      // std::thread
#include <type_traits> // std::enable_if_t
#include <utility>     // std::pair
#include <vector>      // std::vector

namespace neighbour_search
{
const static auto DEFAULT_RECURSION_DEPTH =
    static_cast<std::uint32_t>(std::floor(std::log2(std::thread::hardware_concurrency())));

/// @brief Definition of the point struct
template <typename CoordinateType, std::size_t number_of_dimensions,
          typename = std::enable_if_t<(number_of_dimensions == 2) || (number_of_dimensions == 3)>>
using Point = std::array<CoordinateType, number_of_dimensions>;

template <typename CoordinateType, std::size_t number_of_dimensions> class KDTree final
{
    using PointType = Point<CoordinateType, number_of_dimensions>;
    using KDTreeType = KDTree<CoordinateType, number_of_dimensions>;
    using ReturnType = std::pair<std::size_t, CoordinateType>; // Index + Distance

  public:
    KDTree &operator=(const KDTreeType &other) = delete;
    KDTree(const KDTreeType &other) = delete;
    KDTree &operator=(KDTreeType &&other) noexcept = default;
    KDTree(KDTreeType &&other) noexcept = default;
    KDTree() = delete;

    /// @brief Builds KDTree
    /// @param points List of points
    /// @param threaded Flag that specifies whether threaded execution should be enabled
    /// @throw std::runtime_error if constructed KDTree is empty
    explicit KDTree(const std::vector<PointType> &points, bool threaded = false) : root_(nullptr)
    {
        nodes_.reserve(points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            nodes_.emplace_back(points[i], i);
        }

        if (threaded)
        {
            // Concurrent build
            root_ = buildTreeRecursivelyParallel(nodes_.begin(), nodes_.end(), 0UL);
        }
        else
        {
            // Sequential build
            root_ = buildTreeRecursively(nodes_.begin(), nodes_.end(), 0UL);
        }

        if (root_ == nullptr)
        {
            throw std::runtime_error("KDTree is empty.");
        }
    }

    /// @brief Find a single nearest neighbour
    /// @param target Point of interest
    /// @return Closest point to the target
    [[nodiscard]] ReturnType findNearestNeighbour(const PointType &target) const
    {
        Node *nearest_node = nullptr;
        CoordinateType min_distance_squared = std::numeric_limits<CoordinateType>::max();

        findNearestNeighbourRecursively(root_, target, 0UL, min_distance_squared, nearest_node);

        return std::make_pair(nearest_node->index, min_distance_squared);
    }

    /// @brief Find a single nearest neighbour for each provided target point
    /// @param targets Points of interest
    /// @param neighbours A List of neighbour for each target point
    /// @param threaded Whether to use multithreading to speedup nearest neighbour search
    void findNearestNeighbourForEachTarget(const std::vector<PointType> &targets, std::vector<ReturnType> &neighbours,
                                           bool threaded = false) const
    {
        neighbours.clear();
        neighbours.resize(targets.size());

        std::vector<std::size_t> indices(targets.size());
        std::iota(indices.begin(), indices.end(), 0UL);

        if (threaded)
        {
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](const std::size_t i) -> void {
                Node *nearest_node = nullptr;
                CoordinateType min_distance_squared = std::numeric_limits<CoordinateType>::max();

                findNearestNeighbourRecursively(root_, targets[i], 0UL, min_distance_squared, nearest_node);

                neighbours[i] = std::make_pair(nearest_node->index, min_distance_squared);
            });
        }
        else
        {
            std::for_each(std::execution::seq, indices.begin(), indices.end(), [&](const std::size_t i) -> void {
                Node *nearest_node = nullptr;
                CoordinateType min_distance_squared = std::numeric_limits<CoordinateType>::max();

                findNearestNeighbourRecursively(root_, targets[i], 0UL, min_distance_squared, nearest_node);

                neighbours[i] = std::make_pair(nearest_node->index, min_distance_squared);
            });
        }
    }

    /// @brief Find K Nearest Neighbours closest to target
    /// @param target Target point
    /// @param k Number of neighbours
    /// @param result Result list containing a pair of point index and distance squared to target
    void findKNearestNeighbours(const PointType &target, std::size_t k, std::vector<ReturnType> &result) const
    {
        result.clear();
        if (k == 0)
        {
            return;
        }

        std::priority_queue<ReturnType, std::vector<ReturnType>, CompareDistances> max_heap;

        findKNearestNeighboursRecursively(root_, target, 0UL, k, max_heap);

        result.reserve(max_heap.size());
        while (!max_heap.empty())
        {
            result.emplace_back(max_heap.top());
            max_heap.pop();
        }
    }

    /// @brief Find K Nearest Neighbours closest to target
    /// @param target Target point
    /// @param k Number of neighbours
    /// @param radius_squared Maximum allowed squared distance between the target and one of the nearest neighbours
    /// @param result Result list containing a pair of point index and distance squared to target
    void findKNearestNeighboursWithinRadiusSquared(const PointType &target, std::size_t k,
                                                   CoordinateType radius_squared, std::vector<ReturnType> &result) const
    {
        result.clear();
        if (k == 0)
        {
            return;
        }

        std::priority_queue<ReturnType, std::vector<ReturnType>, CompareDistances> max_heap;

        findKNearestNeighborsWithinRadiusSquaredRecursively(root_, target, 0UL, k, radius_squared, max_heap);

        result.reserve(max_heap.size());
        while (!max_heap.empty())
        {
            result.emplace_back(max_heap.top());
            max_heap.pop();
        }
    }

    /// @brief Find K Nearest Neighbours closest to target
    /// @param target Target point
    /// @param radius_squared Maximum allowed squared distance between the target and one of the nearest neighbours
    /// @param result Result list containing a pair of point index and distance squared to target
    /// @param sort Sort the resulting neighbour points based on their proximity to the target?
    void findAllNearestNeighboursWithinRadiusSquared(const PointType &target, CoordinateType radius_squared,
                                                     std::vector<ReturnType> &result, bool sort = true) const noexcept
    {
        result.clear();

        findAllNeighborsWithinRadiusSquaredRecursively(root_, target, 0UL, radius_squared, result);

        if (sort)
        {
            std::sort(result.begin(), result.end(),
                      [](const ReturnType &d1, const ReturnType &d2) { return (d1.second < d2.second); });
        }
    }

  private:
    /// @brief Node structure containing point, pointer to left and right subtrees
    struct Node final
    {
        explicit Node(const PointType &point, std::size_t index)
            : point(point), index(index), left(nullptr), right(nullptr)
        {
        }
        ~Node()
        {
            left = nullptr;
            right = nullptr;
        }
        PointType point;
        std::size_t index;
        Node *left = nullptr;
        Node *right = nullptr;
    };
    Node *root_ = nullptr;
    std::vector<Node> nodes_;

    /// @brief Distance squared between two points
    /// @param p1 first point
    /// @param p2 second point
    /// @return distance squared between points
    [[nodiscard]] constexpr inline CoordinateType distanceSquared(const PointType &p1,
                                                                  const PointType &p2) const noexcept
    {
        static_assert((number_of_dimensions == 2) || (number_of_dimensions == 3));
        if constexpr (number_of_dimensions == 2)
        {
            const auto dx = p1[0] - p2[0];
            const auto dy = p1[1] - p2[1];
            return (dx * dx) + (dy * dy);
        }
        else
        {
            const auto dx = p1[0] - p2[0];
            const auto dy = p1[1] - p2[1];
            const auto dz = p1[2] - p2[2];
            return (dx * dx) + (dy * dy) + (dz * dz);
        }
    }

    /// @brief Used as a priority queue comparator
    struct CompareDistances
    {
        inline bool operator()(const ReturnType &d1, const ReturnType &d2) const noexcept
        {
            return (d1.second > d2.second);
        }
    };

    /// @brief Recursive sequential build of the KDTree
    /// @param begin begin iterator
    /// @param end end iterator
    /// @param index index between first and last
    /// @return root node
    [[nodiscard]] Node *buildTreeRecursively(typename std::vector<Node>::iterator begin,
                                             typename std::vector<Node>::iterator end, std::size_t index) const noexcept
    {
        if (begin >= end)
        {
            return nullptr;
        }

        auto middle = begin + std::distance(begin, end) / 2;

        std::nth_element(begin, middle, end, [&index](const Node &n1, const Node &n2) -> bool {
            return (n1.point[index] < n2.point[index]);
        });

        index = (index + 1) % number_of_dimensions;

        middle->left = buildTreeRecursively(begin, middle, index);
        middle->right = buildTreeRecursively(middle + 1, end, index);

        return &(*middle);
    }

    /// @brief Recursive parallel build of the KDTree
    /// @param begin begin iterator
    /// @param end end iterator
    /// @param index index between first and last
    /// @return root node
    [[nodiscard]] Node *buildTreeRecursivelyParallel(typename std::vector<Node>::iterator begin,
                                                     typename std::vector<Node>::iterator end, std::size_t index,
                                                     std::uint32_t recursion_depth = 0U) const noexcept
    {
        if (recursion_depth > DEFAULT_RECURSION_DEPTH)
        {
            return buildTreeRecursively(begin, end, index);
        }
        else
        {
            if (begin >= end)
            {
                return nullptr;
            }

            auto middle = begin + std::distance(begin, end) / 2;

            std::nth_element(begin, middle, end, [&index](const Node &n1, const Node &n2) -> bool {
                return (n1.point[index] < n2.point[index]);
            });

            index = (index + 1) % number_of_dimensions;

            auto future = std::async(std::launch::async, &KDTreeType::buildTreeRecursivelyParallel, this,
                                     std::ref(begin), std::ref(middle), index, recursion_depth + 1);
            middle->right = buildTreeRecursivelyParallel(middle + 1, end, index, recursion_depth + 1);
            middle->left = future.get();

            return &(*middle);
        }
    }

    /// @brief Finds closest point to target
    /// @param node Pointer to the root node
    /// @param target Target point
    /// @param index Node index
    /// @param min_distance_squared Distance squared
    /// @param nearest_node Node pointer reference
    void findNearestNeighbourRecursively(const Node *node, const PointType &target, std::size_t index,
                                         CoordinateType &min_distance_squared, Node *&nearest_node) const noexcept
    {
        if (node == nullptr)
        {
            return;
        }

        const auto distance_squared = distanceSquared(target, node->point);
        if (distance_squared <= min_distance_squared)
        {
            min_distance_squared = distance_squared;
            nearest_node = const_cast<Node *>(node);
        }

        const auto delta = node->point[index] - target[index];

        index = (index + 1) % number_of_dimensions;

        const bool is_delta_positive = (delta > 0);
        findNearestNeighbourRecursively(is_delta_positive ? node->left : node->right, target, index,
                                        min_distance_squared, nearest_node);

        if (delta * delta <= min_distance_squared)
        {
            findNearestNeighbourRecursively(is_delta_positive ? node->right : node->left, target, index,
                                            min_distance_squared, nearest_node);
        }
    }

    /// @brief Finds K nearest neighbors closest to target point
    /// @param node Root node
    /// @param target Target point
    /// @param index Node index
    /// @param k Number of neighbours to find
    /// @param max_heap Size of the priority queue
    void findKNearestNeighboursRecursively(
        const Node *node, const PointType &target, std::size_t index, std::size_t k,
        std::priority_queue<ReturnType, std::vector<ReturnType>, CompareDistances> &max_heap) const noexcept
    {
        if (node == nullptr)
        {
            return;
        }

        const auto distance_squared = distanceSquared(target, node->point);

        if (max_heap.size() < k)
        {
            max_heap.emplace(node->index, distance_squared);
        }
        else if (distance_squared < max_heap.top().second)
        {
            max_heap.pop();
            max_heap.emplace(node->index, distance_squared);
        }

        const auto delta = node->point[index] - target[index];

        index = (index + 1) % number_of_dimensions;

        const bool is_delta_positive = (delta > 0);

        findKNearestNeighboursRecursively(is_delta_positive ? node->left : node->right, target, index, k, max_heap);

        if ((delta * delta <= max_heap.top().second) || (max_heap.size() < k))
        {
            findKNearestNeighboursRecursively(is_delta_positive ? node->right : node->left, target, index, k, max_heap);
        }
    }

    /// @brief Finds K nearest neighbors closest to target point within the specified radius squared
    /// @param node Root node
    /// @param target Target point
    /// @param index Node index
    /// @param k Number of neighbours to find
    /// @param radius_squared Radius squared - proximity of neighbours to the target point
    /// @param max_heap Size of the priority queue
    void findKNearestNeighborsWithinRadiusSquaredRecursively(
        const Node *node, const PointType &target, std::size_t index, std::size_t k, CoordinateType radius_squared,
        std::priority_queue<ReturnType, std::vector<ReturnType>, CompareDistances> &max_heap) const noexcept
    {
        if (node == nullptr)
        {
            return;
        }

        const auto distance_squared = distanceSquared(target, node->point);
        if (distance_squared <= radius_squared)
        {
            if (max_heap.size() < k)
            {
                max_heap.emplace(node->index, distance_squared);
            }
            else if (distance_squared < max_heap.top().second)
            {
                max_heap.pop();
                max_heap.emplace(node->index, distance_squared);
            }
        }

        const auto delta = node->point[index] - target[index];

        index = (index + 1) % number_of_dimensions;

        const bool is_delta_positive = (delta > 0);
        findKNearestNeighborsWithinRadiusSquaredRecursively(is_delta_positive ? node->left : node->right, target, index,
                                                            k, radius_squared, max_heap);

        if ((delta * delta <= max_heap.top().second) || (max_heap.size() < k))
        {
            findKNearestNeighborsWithinRadiusSquaredRecursively(is_delta_positive ? node->right : node->left, target,
                                                                index, k, radius_squared, max_heap);
        }
    }

    /// @brief Finds nearest neighbors closest to target point within the specified radius squared
    /// @param node Root node
    /// @param target Target point
    /// @param index Node index
    /// @param radius_squared Radius squared - proximity of neighbours to the target point
    /// @param result Neighbour indices and distances
    void findAllNeighborsWithinRadiusSquaredRecursively(const Node *node, const PointType &target, std::size_t index,
                                                        CoordinateType radius_squared,
                                                        std::vector<ReturnType> &result) const noexcept
    {
        if (node == nullptr)
        {
            return;
        }

        const auto distance_squared = distanceSquared(target, node->point);
        if (distance_squared <= radius_squared)
        {
            result.emplace_back(node->index, distance_squared);
        }

        const auto delta = node->point[index] - target[index];
        index = (index + 1) % number_of_dimensions;

        const bool is_delta_positive = (delta > 0);
        findAllNeighborsWithinRadiusSquaredRecursively(is_delta_positive ? node->left : node->right, target, index,
                                                       radius_squared, result);

        if (delta * delta <= radius_squared)
        {
            findAllNeighborsWithinRadiusSquaredRecursively(is_delta_positive ? node->right : node->left, target, index,
                                                           radius_squared, result);
        }
    }
};
} // namespace neighbour_search

#endif // KDTREE_HPP