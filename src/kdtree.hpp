#ifndef LIDAR_PROCESSING__KDTREE_HPP
#define LIDAR_PROCESSING__KDTREE_HPP

#include "priority_queue.hpp"
#include "vector.hpp"

// STL
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

namespace lidar_processing
{
template <typename T, std::uint8_t Dim> using Point = T[Dim];

template <typename T, std::uint8_t Dim> class KDTree final
{
    using PointT = Point<T, Dim>;
    using KDTreeT = KDTree<T, Dim>;
    using RetT = std::pair<std::uint32_t, T>;

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
    KDTree &operator=(const KDTreeT &other) = delete;
    KDTree(const KDTreeT &other) = delete;
    KDTree &operator=(KDTreeT &&other) noexcept = default;
    KDTree(KDTreeT &&other) noexcept = default;

    KDTree(bool sort = false, std::uint8_t num_threads = 1);

    void reserve(std::uint32_t num_pts = 200'000U);
    void rebuild(const containers::Vector<PointT> &points);
    void k_nearest(const PointT &target, std::uint32_t num_neigh, containers::Vector<RetT> &neigh);
    void radius_search(const PointT &target, T dist_sqr, containers::Vector<RetT> &neigh);

  private:
    bool sort_;
    bool threaded_;
    std::uint8_t num_threads_;

    Node *root_{nullptr};
    containers::Vector<Node> nodes_;

    template <std::uint8_t N = 0> inline static constexpr T dist_sqr_impl(const PointT &a, const PointT &b) noexcept;
    inline static constexpr T dist_sqr(const PointT &a, const PointT &b) noexcept;
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
}

template <typename T, std::uint8_t Dim> void KDTree<T, Dim>::rebuild(const containers::Vector<PointT> &points)
{
}

template <typename T, std::uint8_t Dim>
void KDTree<T, Dim>::k_nearest(const PointT &target, std::uint32_t num_neigh, containers::Vector<RetT> &neigh)
{
}

template <typename T, std::uint8_t Dim>
void KDTree<T, Dim>::radius_search(const PointT &target, T dist_sqr, containers::Vector<RetT> &neigh)
{
}

} // namespace lidar_processing

#endif // LIDAR_PROCESSING__KDTREE_HPP
