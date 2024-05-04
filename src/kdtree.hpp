#ifndef LIDAR_PROCESSING__KDTREE_HPP
#define LIDAR_PROCESSING__KDTREE_HPP

#include "priority_queue.hpp"
#include "stack.hpp"
#include "vector.hpp"

// STL
#include <algorithm> // std::sort, std::nth_element
#include <array>     // std::array
#include <cstdint>   // std::size_t
#include <stdexcept> // std::runtime_error
#include <utility>   // std::pair

namespace lidar_processing
{
template <typename T, std::uint8_t Dim> using Point = std::array<T, Dim>;

template <typename T, std::uint8_t Dim> class KDTree final
{
    using PointT = Point<T, Dim>;
    using KDTreeT = KDTree<T, Dim>;

    struct Node final
    {
        PointT point{};
        std::uint32_t index{0U};
        Node *left{nullptr};
        Node *right{nullptr};

        Node() = default;
        explicit Node(const PointT &point, std::uint32_t index) : point(point), index(index)
        {
        }
    };

  public:
    using RetT = std::pair<std::uint32_t, T>;

    struct RetTCompare final
    {
        bool operator()(const RetT &a, const RetT &b) const noexcept
        {
            return a.second < b.second;
        }
    };

    KDTree &operator=(const KDTreeT &other) = delete;
    KDTree(const KDTreeT &other) = delete;
    KDTree &operator=(KDTreeT &&other) noexcept = default;
    KDTree(KDTreeT &&other) noexcept = default;

    KDTree(bool sort = false, std::uint8_t num_threads = 1);

    void reserve(std::uint32_t num_pts = 200'000U);
    void rebuild(const containers::Vector<PointT> &points);
    void k_nearest(const PointT &target, std::uint32_t num_neigh, containers::Vector<RetT> &neigh);
    void radius_search(const PointT &target, T proximity_sqr, containers::Vector<RetT> &neigh);

    inline static constexpr T dist_sqr(const PointT &a, const PointT &b) noexcept;

  private:
    struct RebuildStack final
    {
        typename containers::Vector<Node>::iterator begin{nullptr};
        typename containers::Vector<Node>::iterator end{nullptr};
        Node **node_ref{nullptr};
        std::uint32_t depth{0U};

        RebuildStack() = default;
        explicit RebuildStack(typename containers::Vector<Node>::iterator begin,
                              typename containers::Vector<Node>::iterator end, Node **node_ref, std::uint32_t depth)
            : begin(begin), end(end), node_ref(node_ref), depth(depth)
        {
        }
    };

    struct KNearestStack final
    {
        const Node *node_ptr{nullptr};
        std::uint32_t index{0U};
        bool first_visit{true};

        KNearestStack() = default;
        KNearestStack(const Node *node_ptr, std::uint32_t index, bool first_visit)
            : node_ptr(node_ptr), index(index), first_visit(first_visit)
        {
        }
    };

    struct RadiusSearchStack final
    {
        const Node *node_ptr{nullptr};
        std::uint32_t index{0U};

        RadiusSearchStack() = default;
        RadiusSearchStack(const Node *node_ptr, std::uint32_t index) : node_ptr(node_ptr), index(index)
        {
        }
    };

    bool sort_;
    bool threaded_;
    std::uint8_t num_threads_;

    Node *root_{nullptr};
    containers::Vector<Node> nodes_;
    containers::Stack<RebuildStack> rebuild_stack_;
    containers::Stack<KNearestStack> k_nearest_stack_;
    containers::Stack<RadiusSearchStack> radius_search_stack_;
    containers::PriorityQueue<RetT, RetTCompare> min_heap_;

    template <std::uint8_t N = 0> inline static constexpr T dist_sqr_impl(const PointT &a, const PointT &b) noexcept;
};

template <typename T, std::uint8_t Dim>
KDTree<T, Dim>::KDTree(bool sort, std::uint8_t num_threads)
    : sort_(sort), threaded_(num_threads > 1), num_threads_(num_threads)
{
    reserve();
}

template <typename T, std::uint8_t Dim>
template <std::uint8_t N>
inline constexpr T KDTree<T, Dim>::dist_sqr_impl(const PointT &a, const PointT &b) noexcept
{
    if constexpr (N < Dim)
    {
        return (a[N] - b[N]) * (a[N] - b[N]) + dist_sqr_impl<N + 1>(a, b);
    }
    else
    {
        return 0;
    }
}

template <typename T, std::uint8_t Dim>
inline constexpr T KDTree<T, Dim>::dist_sqr(const PointT &a, const PointT &b) noexcept
{
    return dist_sqr_impl(a, b);
}

template <typename T, std::uint8_t Dim> void KDTree<T, Dim>::reserve(std::uint32_t num_pts)
{
    nodes_.reserve(num_pts);
    rebuild_stack_.reserve(num_pts);
    k_nearest_stack_.reserve(num_pts);
    radius_search_stack_.reserve(num_pts);
    min_heap_.reserve(num_pts);
}

template <typename T, std::uint8_t Dim> void KDTree<T, Dim>::rebuild(const containers::Vector<PointT> &points)
{
    nodes_.clear();
    root_ = nullptr;

    if (points.empty())
    {
        return;
    }

    nodes_.reserve(points.size());
    for (std::uint32_t i = 0U; i < points.size(); ++i)
    {
        nodes_.emplace_back(points[i], i);
    }

    rebuild_stack_.emplace(nodes_.begin(), nodes_.end(), &root_, 0U);

    while (rebuild_stack_.size() > 0U)
    {
        auto [begin, end, node_ref, depth] = std::move(rebuild_stack_.top());
        rebuild_stack_.try_pop();

        if (begin >= end)
        {
            continue;
        }

        const auto axis = depth % Dim;
        auto mid = begin + (end - begin) / 2;

        std::nth_element(begin, mid, end,
                         [axis](const Node &a, const Node &b) -> bool { return a.point[axis] < b.point[axis]; });

        *node_ref = &(*mid);

        if (mid > begin)
        {
            rebuild_stack_.emplace(begin, mid, &((*node_ref)->left), depth + 1);
        }

        if ((mid + 1) < end)
        {
            rebuild_stack_.emplace(mid + 1, end, &((*node_ref)->right), depth + 1);
        }
    }

    if (root_ == nullptr)
    {
        throw std::runtime_error("KDTree is empty");
    }
}

template <typename T, std::uint8_t Dim>
void KDTree<T, Dim>::k_nearest(const PointT &target, std::uint32_t num_neigh, containers::Vector<RetT> &neigh)
{
    neigh.clear();

    if ((num_neigh == 0) || (root_ == nullptr))
    {
        return;
    }

    k_nearest_stack_.emplace(root_, 0U, true);

    while (k_nearest_stack_.size() > 0U)
    {
        auto [node, index, first_visit] = std::move(k_nearest_stack_.top());
        k_nearest_stack_.pop();

        if (node == nullptr)
        {
            continue;
        }

        if (first_visit)
        {
            const auto dist = dist_sqr(target, node->point);

            if (min_heap_.size() < num_neigh)
            {
                min_heap_.emplace(node->index, dist);
            }
            else if (dist < min_heap_.top().second)
            {
                min_heap_.pop();
                min_heap_.emplace(node->index, dist);
            }

            const auto next_index = (index + 1U) % Dim;
            const auto delta = node->point[index] - target[index];
            const auto next_branch = (delta > 0) ? node->left : node->right;

            k_nearest_stack_.emplace(node, index, false);
            k_nearest_stack_.emplace(next_branch, next_index, true);
        }
        else
        {
            const auto delta = node->point[index] - target[index];
            const auto next_index = (index + 1U) % Dim;
            const auto opposite_branch = (delta > 0) ? node->right : node->left;

            if ((delta * delta <= min_heap_.top().second) || (min_heap_.size() < num_neigh))
            {
                k_nearest_stack_.emplace(opposite_branch, next_index, true);
            }
        }
    }

    neigh.reserve(min_heap_.size());
    while (min_heap_.size() > 0U)
    {
        neigh.emplace_back(std::move(min_heap_.top()));
        min_heap_.pop();
    }
    std::reverse(neigh.begin(), neigh.end());
}

template <typename T, std::uint8_t Dim>
void KDTree<T, Dim>::radius_search(const PointT &target, T proximity_sqr, containers::Vector<RetT> &neigh)
{
    neigh.clear();

    if (root_ == nullptr)
    {
        return;
    }

    radius_search_stack_.emplace(root_, 0U);

    while (radius_search_stack_.size() > 0U)
    {
        auto [node, index] = std::move(radius_search_stack_.top());
        radius_search_stack_.try_pop();

        if (node == nullptr)
        {
            continue;
        }

        const auto dist = dist_sqr(target, node->point);
        if (dist <= proximity_sqr)
        {
            neigh.emplace_back(node->index, dist);
        }

        const auto next_index = (index + 1) % Dim;
        const auto delta = node->point[index] - target[index];
        const auto abs_delta_sqr = delta * delta;

        if (abs_delta_sqr <= proximity_sqr)
        {
            radius_search_stack_.emplace(node->right, next_index);
            radius_search_stack_.emplace(node->left, next_index);
        }
        else
        {
            const auto next_branch = (delta > 0) ? node->left : node->right;
            radius_search_stack_.emplace(next_branch, next_index);
        }
    }

    if (sort_)
    {
        std::sort(neigh.begin(), neigh.end(),
                  [](const RetT &ret_1, const RetT &ret_2) -> bool { return (ret_1.second < ret_2.second); });
    }
}

} // namespace lidar_processing

#endif // LIDAR_PROCESSING__KDTREE_HPP
