//
// Author: Stanislaw Adaszewski, 2019
// C++ port from https://github.com/mapbox/concaveman (js)
//
// Comments from js repo added by wheeled
//

#pragma once
#ifndef CONCAVE_HULL_HPP_
#define CONCAVE_HULL_HPP_

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

extern "C"
{
#include <assert.h>
#include <stdlib.h>
}

namespace lidar_processing
{

template <typename T> inline bool numberIsZero(const T &number)
{
    return (std::abs(number) < std::numeric_limits<T>::min());
};

template <typename T> inline bool numberIsNegative(const T &number)
{
    return std::signbit(number);
};

template <typename T> inline bool numbersAreEqual(const T &number1, const T &number2)
{
    return (std::abs(number1 - number2) < std::numeric_limits<T>::min());
}

template <class T> class CompareFirst
{
  public:
    bool operator()(const T &a, const T &b)
    {
        return (std::get<0>(a) < std::get<0>(b));
    }
};

template <class T> inline T orient2d(const std::array<T, 2> &p1, const std::array<T, 2> &p2, const std::array<T, 2> &p3)
{
    return ((p2[1] - p1[1]) * (p3[0] - p2[0]) - (p2[0] - p1[0]) * (p3[1] - p2[1]));
}

// check if the edges (p1,q1) and (p2,q2) intersect
template <class T>
inline bool intersects(const std::array<T, 2> &p1, const std::array<T, 2> &q1, const std::array<T, 2> &p2,
                       const std::array<T, 2> &q2)
{
    return (p1[0] != q2[0] || p1[1] != q2[1]) && (q1[0] != p2[0] || q1[1] != p2[1]) &&
           (orient2d(p1, q1, p2) > 0) != (orient2d(p1, q1, q2) > 0) &&
           (orient2d(p2, q2, p1) > 0) != (orient2d(p2, q2, q1) > 0);
}

// square distance between 2 points
template <class T> T squarePointToPointDistance(const std::array<T, 2> &p1, const std::array<T, 2> &p2)
{
    auto dx = p1[0] - p2[0];
    auto dy = p1[1] - p2[1];
    return dx * dx + dy * dy;
}

// square distance from a point to a segment
template <class T>
T squarePointToSegmentDistance(const std::array<T, 2> &p, const std::array<T, 2> &p1, const std::array<T, 2> &p2)
{

    auto x = p1[0];
    auto y = p1[1];
    auto dx = p2[0] - x;
    auto dy = p2[1] - y;

    if ((!numberIsZero(dx)) || (!numberIsZero(dy)))
    {
        auto t = ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);
        if (t > 1)
        {
            x = p2[0];
            y = p2[1];
        }
        else if (t > 0)
        {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = p[0] - x;
    dy = p[1] - y;

    return dx * dx + dy * dy;
}

// segment to segment distance, ported from http://geomalgorithms.com/a07-_distance.html by Dan Sunday
template <class T>
T squareSegmentToSegmentDistance(const T &x0, const T &y0, const T &x1, const T &y1, const T &x2, const T &y2,
                                 const T &x3, const T &y3)
{
    auto ux = x1 - x0;
    auto uy = y1 - y0;
    auto vx = x3 - x2;
    auto vy = y3 - y2;
    auto wx = x0 - x2;
    auto wy = y0 - y2;
    auto a = ux * ux + uy * uy;
    auto b = ux * vx + uy * vy;
    auto c = vx * vx + vy * vy;
    auto d = ux * wx + uy * wy;
    auto e = vx * wx + vy * wy;
    auto D = a * c - b * b;

    T sc, sN, tc, tN;
    auto sD = D;
    auto tD = D;

    auto zero_ = std::numeric_limits<T>::min();

    if (numberIsZero(D)) // check if the number is equal to zero
    {
        sN = 0;
        sD = 1;
        tN = e;
        tD = c;
    }
    else
    {
        sN = b * e - c * d;
        tN = a * e - b * d;
        if (numberIsNegative(sN)) // Check if the number is negative
        {
            sN = 0;
            tN = e;
            tD = c;
        }
        else if (sN > sD)
        {
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0)
    {
        tN = 0;
        if (numberIsNegative(-d)) // check if the number is negative
        {
            sN = 0;
        }
        else if ((-d) > a)
        {
            sN = sD;
        }
        else
        {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD)
    {
        tN = tD;
        if (numberIsNegative(-d + b)) // check if the sum is negative
        {
            sN = 0;
        }
        else if ((-d + b) > a)
        {
            sN = sD;
        }
        else
        {
            sN = -d + b;
            sD = a;
        }
    }

    sc = (numberIsZero(sN) ? 0 : sN / sD);
    tc = (numberIsZero(tN) ? 0 : tN / tD);

    auto cx = (1 - sc) * x0 + sc * x1;
    auto cy = (1 - sc) * y0 + sc * y1;
    auto cx2 = (1 - tc) * x2 + tc * x3;
    auto cy2 = (1 - tc) * y2 + tc * y3;
    auto dx = cx2 - cx;
    auto dy = cy2 - cy;

    return dx * dx + dy * dy;
}

template <class T, int DIM, int MAX_CHILDREN, class DATA> class RTree
{
  public:
    using type = RTree<T, DIM, MAX_CHILDREN, DATA>;
    using const_type = const type;
    using type_ptr = type *;
    using type_const_ptr = const type *;
    using bounds_type = std::array<T, DIM * 2>;
    using data_type = DATA;

    RTree() : m_is_leaf(false), m_data()
    {
        for (auto i = 0; i < DIM; ++i)
        {
            m_bounds[i] = std::numeric_limits<T>::max();
            m_bounds[i + DIM] = std::numeric_limits<T>::min();
        }
    }

    RTree(data_type data, const bounds_type &bounds) : m_is_leaf(true), m_data(data), m_bounds(bounds)
    {
        for (auto i = 0; i < DIM; i++)
            if (bounds[i] > bounds[i + DIM])
            {
                throw std::runtime_error("Bounds minima have to be less than maxima");
            }
    }

    void insert(data_type data, const bounds_type &bounds)
    {
        if (m_is_leaf)
        {
            throw std::runtime_error("Cannot insert into leaves");
        }

        m_bounds = updated_bounds(bounds);
        if (m_children.size() < MAX_CHILDREN)
        {
            auto r = std::make_unique<type>(data, bounds);
            m_children.push_back(std::move(r));
            return;
        }

        std::reference_wrapper<type> best_child = *m_children.begin()->get();
        auto best_volume = volume(best_child.get().updated_bounds(bounds));
        for (auto it = ++m_children.begin(); it != m_children.end(); ++it)
        {
            auto v = volume((*it)->updated_bounds(bounds));
            if (v < best_volume)
            {
                best_volume = v;
                best_child = *it->get();
            }
        }
        if (!best_child.get().is_leaf())
        {
            best_child.get().insert(data, bounds);
            return;
        }

        auto leaf = std::make_unique<type>(best_child.get().data(), best_child.get().bounds());
        best_child.get().m_is_leaf = false;
        best_child.get().m_data = data_type();
        best_child.get().m_children.push_back(std::move(leaf));
        best_child.get().insert(data, bounds);
    }

    void intersection(const bounds_type &bounds, std::vector<std::reference_wrapper<const_type>> &res) const
    {
        if (!intersects(bounds))
        {
            return;
        }
        if (m_is_leaf)
        {
            res.push_back(*this);
            return;
        }
        for (auto &ch : m_children)
        {
            ch->intersection(bounds, res);
        }
    }

    std::vector<std::reference_wrapper<const_type>> intersection(const bounds_type &bounds) const
    {
        std::vector<std::reference_wrapper<const_type>> res;
        intersection(bounds, res);
        return res;
    }

    bool intersects(const bounds_type &bounds) const
    {
        for (auto i = 0; i < DIM; ++i)
        {
            if ((m_bounds[i] > bounds[i + DIM]) || (m_bounds[i + DIM] < bounds[i]))
            {
                return (false);
            }
        }
        return (true);
    }

    void erase(data_type data, const bounds_type &bounds)
    {
        if (m_is_leaf)
        {
            throw std::runtime_error("Cannot erase from leaves");
        }

        if (!intersects(bounds))
        {
            return;
        }

        for (auto it = m_children.begin(); it != m_children.end();)
        {
            if (!((*it)->m_is_leaf))
            {
                (*it)->erase(data, bounds);
                ++it;
            }
            else if (((*it)->m_data == data) && ((*it)->m_bounds == bounds))
            {
                m_children.erase(it++);
            }
            else
            {
                ++it;
            }
        }
    }

    bounds_type updated_bounds(const bounds_type &child_bounds) const
    {
        bounds_type res;
        for (auto i = 0; i < DIM; ++i)
        {
            res[i] = std::min(child_bounds[i], m_bounds[i]);
            res[i + DIM] = std::max(child_bounds[i + DIM], m_bounds[i + DIM]);
        }
        return res;
    }

    static T volume(const bounds_type &bounds)
    {
        T res = 1;
        for (auto i = 0; i < DIM; ++i)
        {
            auto delta = bounds[i + DIM] - bounds[i];
            res *= delta;
        }
        return res;
    }

    const bounds_type &bounds() const
    {
        return m_bounds;
    }

    bool is_leaf() const
    {
        return m_is_leaf;
    }

    data_type data() const
    {
        return m_data;
    }

    const std::list<std::unique_ptr<type>> &children() const
    {
        return m_children;
    }

  private:
    bool m_is_leaf;
    data_type m_data;
    std::list<std::unique_ptr<type>> m_children;
    bounds_type m_bounds;
};

template <class T> struct Node
{
    using type = Node<T>;
    using type_ptr = type *;
    using point_type = std::array<T, 2>;

    Node() : p(), min_x(), min_y(), max_x(), max_y()
    {
    }

    Node(const point_type &p) : Node()
    {
        this->p = p;
    }

    point_type p;
    T min_x;
    T min_y;
    T max_x;
    T max_y;
};

template <class T> class CircularList;

template <class T> class CircularElement
{
  public:
    using type = CircularElement<T>;
    using ptr_type = type *;

    template <class... Args> CircularElement<T>(Args &&...args) : m_data(std::forward<Args>(args)...)
    {
    }

    T &data()
    {
        return m_data;
    }

    template <class... Args> CircularElement<T> *insert(Args &&...args)
    {
        auto elem = new CircularElement<T>(std::forward<Args>(args)...);
        elem->m_prev = this;
        elem->m_next = m_next;
        m_next->m_prev = elem;
        m_next = elem;
        return elem;
    }

    CircularElement<T> *prev()
    {
        return m_prev;
    }

    CircularElement<T> *next()
    {
        return m_next;
    }

  private:
    T m_data;
    CircularElement<T> *m_prev = nullptr;
    CircularElement<T> *m_next = nullptr;

    friend class CircularList<T>;
};

template <class T> class CircularList
{
  public:
    using element_type = CircularElement<T>;

    CircularList() : m_last(nullptr)
    {
    }

    ~CircularList()
    {
        auto node = m_last;
        while (true)
        {
            auto tmp = node;
            node = node->m_next;
            delete tmp;
            if (node == m_last)
            {
                break;
            }
        }
        m_last = nullptr;
    }

    template <class... Args> CircularElement<T> *insert(element_type *prev, Args &&...args)
    {
        auto elem = new CircularElement<T>(std::forward<Args>(args)...);

        if ((prev == nullptr) && (m_last != nullptr))
        {
            throw std::runtime_error("Once the list is non-empty you must specify where to insert");
        }

        if (prev == nullptr)
        {
            elem->m_prev = elem->m_next = elem;
        }
        else
        {
            elem->m_prev = prev;
            elem->m_next = prev->m_next;
            prev->m_next->m_prev = elem;
            prev->m_next = elem;
        }

        m_last = elem;

        return elem;
    }

  private:
    element_type *m_last = nullptr;
};

// update the bounding box of a node's edge
template <class T> void updateBBox(typename CircularElement<T>::ptr_type elem)
{
    auto &node(elem->data());
    auto p1 = node.p;
    auto p2 = elem->next()->data().p;
    node.min_x = std::min(p1[0], p2[0]);
    node.min_y = std::min(p1[1], p2[1]);
    node.max_x = std::max(p1[0], p2[0]);
    node.max_y = std::max(p1[1], p2[1]);
}

template <class T, int MAX_CHILDREN>
std::vector<std::array<T, 2>> Concaveman(
    const std::vector<std::array<T, 2>> &points,
    // start with a convex hull of the points
    const std::vector<int> &hull,
    // a relative measure of concavity; higher value means simpler hull
    T concavity = 2,
    // when a segment goes below this length threshold, it won't be drilled down further
    T length_threshold = 0)
{

    using node_type = Node<T>;
    using point_type = std::array<T, 2>;
    using circ_elem_type = CircularElement<node_type>;
    using circ_list_type = CircularList<node_type>;
    using circ_elem_ptr_type = circ_elem_type *;

    // exit if hull includes all points already
    if (hull.size() == points.size())
    {
        std::vector<point_type> res;
        for (auto &i : hull)
        {
            res.push_back(points[i]);
        }
        return res;
    }

    // index the points with an R-tree
    RTree<T, 2, MAX_CHILDREN, point_type> tree;

    for (auto &p : points)
    {
        tree.insert(p, {p[0], p[1], p[0], p[1]});
    }

    circ_list_type circList;
    circ_elem_ptr_type last = nullptr;

    std::list<circ_elem_ptr_type> queue;

    // turn the convex hull into a linked list and populate the initial edge queue with the nodes
    for (auto &idx : hull)
    {
        auto &p = points[idx];
        tree.erase(p, {p[0], p[1], p[0], p[1]});
        last = circList.insert(last, p);
        queue.push_back(last);
    }

    // loops through the hull?  why?
    for (auto elem = last->next();; elem = elem->next())
    {
        if (elem == last)
        {
            break;
        }
    }

    // index the segments with an R-tree (for intersection checks)
    RTree<T, 2, MAX_CHILDREN, circ_elem_ptr_type> seg_tree;
    for (auto &elem : queue)
    {
        auto &node(elem->data());
        updateBBox<node_type>(elem);
        seg_tree.insert(elem, {node.min_x, node.min_y, node.max_x, node.max_y});
    }

    auto sqConcavity = concavity * concavity;
    auto sqLenThreshold = length_threshold * length_threshold;

    // process edges one by one
    while (!queue.empty())
    {
        auto elem = *queue.begin();
        queue.pop_front();

        auto a = elem->prev()->data().p;
        auto b = elem->data().p;
        auto c = elem->next()->data().p;
        auto d = elem->next()->next()->data().p;

        // skip the edge if it's already short enough
        auto sqLen = squarePointToPointDistance(b, c);
        if (sqLen < sqLenThreshold)
        {
            continue;
        }

        auto maxSqLen = sqLen / sqConcavity;

        // find the best connection point for the current edge to flex inward to
        bool ok;
        auto p = findCandidate(tree, a, b, c, d, maxSqLen, seg_tree, ok);

        // if we found a connection and it satisfies our concavity measure
        if (ok && (std::min(squarePointToPointDistance(p, b), squarePointToPointDistance(p, c)) <= maxSqLen))
        {
            // connect the edge endpoints through this point and add 2 new edges to the queue
            queue.push_back(elem);
            queue.push_back(elem->insert(p));

            // update point and segment indexes
            auto &node = elem->data();
            auto &next = elem->next()->data();

            tree.erase(p, {p[0], p[1], p[0], p[1]});
            seg_tree.erase(elem, {node.min_x, node.min_y, node.max_x, node.max_y});

            updateBBox<node_type>(elem);
            updateBBox<node_type>(elem->next());

            seg_tree.insert(elem, {node.min_x, node.min_y, node.max_x, node.max_y});
            seg_tree.insert(elem->next(), {next.min_x, next.min_y, next.max_x, next.max_y});
        }
    }

    // convert the resulting hull linked list to an array of points
    std::vector<point_type> concave;
    for (auto elem = last->next();; elem = elem->next())
    {
        concave.push_back(elem->data().p);
        if (elem == last)
        {
            break;
        }
    }

    return concave;
}

template <class T, int MAX_CHILDREN>
std::array<T, 2> findCandidate(const RTree<T, 2, MAX_CHILDREN, std::array<T, 2>> &tree, const std::array<T, 2> &a,
                               const std::array<T, 2> &b, const std::array<T, 2> &c, const std::array<T, 2> &d,
                               T maxDist,
                               const RTree<T, 2, MAX_CHILDREN, typename CircularElement<Node<T>>::ptr_type> &seg_tree,
                               bool &ok)
{

    using point_type = std::array<T, 2>;
    using circ_elem_type = CircularElement<Node<T>>;
    using tree_type = RTree<T, 2, MAX_CHILDREN, std::array<T, 2>>;
    using const_tree_type = const tree_type;
    using tree_ref_type = std::reference_wrapper<const_tree_type>;
    using tuple_type = std::tuple<T, tree_ref_type>;

    ok = false;

    std::priority_queue<tuple_type, std::vector<tuple_type>, CompareFirst<tuple_type>> queue;
    std::reference_wrapper<const_tree_type> node = tree;

    // search through the point R-tree with a depth-first search using a priority queue
    // in the order of distance to the edge (b, c)
    while (true)
    {
        for (auto &child : node.get().children())
        {

            auto bounds = child->bounds();
            point_type pt = {bounds[0], bounds[1]};

            auto dist = child->is_leaf() ? squarePointToSegmentDistance(pt, b, c)
                                         : squareSegmentToBoundingBoxDistance(b, c, *child);
            if (dist > maxDist)
            {
                continue; // skip the node if it's farther than we ever need
            }

            queue.push(tuple_type(-dist, *child));
        }

        while ((!queue.empty()) && (std::get<1>(queue.top()).get().is_leaf()))
        {
            auto item = queue.top();
            queue.pop();

            auto bounds = std::get<1>(item).get().bounds();
            point_type p = {bounds[0], bounds[1]};

            // skip all points that are as close to adjacent edges (a,b) and (c,d),
            // and points that would introduce self-intersections when connected
            auto d0 = squarePointToSegmentDistance(p, a, b);
            auto d1 = squarePointToSegmentDistance(p, c, d);

            if ((-std::get<0>(item) < d0) && (-std::get<0>(item) < d1) && noIntersections(b, p, seg_tree) &&
                noIntersections(c, p, seg_tree))
            {

                ok = true;
                return (std::get<1>(item).get().data());
            }
        }

        if (queue.empty())
        {
            break;
        }

        node = std::get<1>(queue.top());
        queue.pop();
    }

    return point_type();
}

// square distance from a segment bounding box to the given one
template <class T, int MAX_CHILDREN, class USER_DATA>
T squareSegmentToBoundingBoxDistance(const std::array<T, 2> &a, const std::array<T, 2> &b,
                                     const RTree<T, 2, MAX_CHILDREN, USER_DATA> &bbox)
{

    if (inside(a, bbox) || inside(b, bbox))
    {
        return 0;
    }

    auto &bounds = bbox.bounds();
    auto min_x = bounds[0];
    auto min_y = bounds[1];
    auto max_x = bounds[2];
    auto max_y = bounds[3];

    auto d1 = squareSegmentToSegmentDistance(a[0], a[1], b[0], b[1], min_x, min_y, max_x, min_y);
    if (numberIsZero(d1))
    {
        return 0;
    }

    auto d2 = squareSegmentToSegmentDistance(a[0], a[1], b[0], b[1], min_x, min_y, min_x, max_y);
    if (numberIsZero(d2))
    {
        return 0;
    }

    auto d3 = squareSegmentToSegmentDistance(a[0], a[1], b[0], b[1], max_x, min_y, max_x, max_y);
    if (numberIsZero(d3))
    {
        return 0;
    }

    auto d4 = squareSegmentToSegmentDistance(a[0], a[1], b[0], b[1], min_x, max_y, max_x, max_y);
    if (numberIsZero(d4))
    {
        return 0;
    }

    return std::min(std::min(d1, d2), std::min(d3, d4));
}

template <class T, int MAX_CHILDREN, class USER_DATA>
bool inside(const std::array<T, 2> &a, const RTree<T, 2, MAX_CHILDREN, USER_DATA> &bbox)
{

    auto &bounds = bbox.bounds();

    auto min_x = bounds[0];
    auto min_y = bounds[1];
    auto max_x = bounds[2];
    auto max_y = bounds[3];

    auto res = (a[0] >= min_x) && (a[0] <= max_x) && (a[1] >= min_y) && (a[1] <= max_y);
    return res;
}

// check if the edge (a,b) doesn't intersect any other edges
template <class T, int MAX_CHILDREN>
bool noIntersections(const std::array<T, 2> &a, const std::array<T, 2> &b,
                     const RTree<T, 2, MAX_CHILDREN, typename CircularElement<Node<T>>::ptr_type> &seg_tree)
{

    auto min_x = std::min(a[0], b[0]);
    auto min_y = std::min(a[1], b[1]);
    auto max_x = std::max(a[0], b[0]);
    auto max_y = std::max(a[1], b[1]);

    auto intersections = seg_tree.intersection({min_x, min_y, max_x, max_y});

    for (const decltype(seg_tree) &ch : intersections)
    {
        auto elem = ch.data();

        if (intersects(elem->data().p, elem->next()->data().p, a, b))
        {
            return (false);
        }
    }

    return (true);
}
} // namespace lidar_processing

#endif // CONCAVE_HULL_HPP_