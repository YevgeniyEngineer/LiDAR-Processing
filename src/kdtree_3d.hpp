

#ifndef LIDAR_PROCESSING__KDTREE_3D_HPP
#define LIDAR_PROCESSING__KDTREE_3D_HPP

// Internal
#include "cartesian_vector_3d_adapter.hpp"
#include "priority_queue.hpp"
#include "stack.hpp"
#include "vector.hpp"

// STL
#include <algorithm>
#include <cstdint>
#include <utility>

namespace lidar_processing
{
using namespace containers;

class KDTree3D final
{
    using Point3D = CartesianVector3DAdapter::Point3D;

    struct Node final
    {
        Point3D point{nullptr};
        std::uint32_t index{0U};
        Node *left{nullptr};
        Node *right{nullptr};

        Node() = default;
        explicit Node(float *ptr, std::uint32_t index) : point(ptr), index(index)
        {
        }
    };

  public:
    using RetT = std::pair<std::uint32_t, float>;

    struct RetTCompare final
    {
        bool operator()(const RetT &a, const RetT &b) const noexcept
        {
            return a.second < b.second;
        }
    };

    KDTree3D(bool sort = false, std::uint8_t num_threads = 1);

    static constexpr float dist_sqr(const Point3D &a, const Point3D &b) noexcept;

    void reserve(std::uint32_t num_pts = 200'000U);
    void rebuild(const float *coordinates, std::uint32_t num_pts); // x1, y1, z1, x2, y2, z2, ...
    void k_nearest(const float *target, std::uint32_t num_neigh, Vector<RetT> &neigh);
    void radius_search(const float *target, float proximity_sqr, Vector<RetT> &neigh);

  private:
    struct RebuildStack final
    {
        typename Vector<Node>::iterator begin{nullptr};
        typename Vector<Node>::iterator end{nullptr};
        Node **node_ref{nullptr};
        std::uint32_t depth{0U};

        RebuildStack() = default;
        explicit RebuildStack(typename Vector<Node>::iterator begin, typename Vector<Node>::iterator end,
                              Node **node_ref, std::uint32_t depth)
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
    Vector<Node> nodes_;
    Stack<RebuildStack> rebuild_stack_;
    Stack<KNearestStack> k_nearest_stack_;
    Stack<RadiusSearchStack> radius_search_stack_;
    PriorityQueue<RetT, RetTCompare> min_heap_;
};

inline KDTree3D::KDTree3D(bool sort, std::uint8_t num_threads)
    : sort_(sort), threaded_(num_threads > 1), num_threads_(num_threads)
{
    reserve();
}

inline constexpr float KDTree3D::dist_sqr(const Point3D &a, const Point3D &b) noexcept
{
    const float dx = b.x() - a.x();
    const float dy = b.y() - a.y();
    const float dz = b.z() - a.z();

    return dx * dx + dy * dy + dz * dz;
}

inline void KDTree3D::reserve(std::uint32_t num_pts)
{
    nodes_.reserve(num_pts);
    rebuild_stack_.reserve(num_pts);
    k_nearest_stack_.reserve(num_pts);
    radius_search_stack_.reserve(num_pts);
    min_heap_.reserve(num_pts);
}

inline void KDTree3D::rebuild(const float *coordinates, std::uint32_t num_pts)
{
    // TODO
}

inline void KDTree3D::k_nearest(const float *target, std::uint32_t num_neigh, Vector<RetT> &neigh)
{
    // TODO
}

inline void KDTree3D::radius_search(const float *target, float proximity_sqr, Vector<RetT> &neigh)
{
    // TODO
}

} // namespace lidar_processing

#endif // LIDAR_PROCESSING__KDTREE_3D_HPP
