// A port of [Delaunator](https://github.com/mapbox/delaunator) and https://github.com/delfrrr/delaunator-cpp

#ifndef DELAUNATOR
#define DELAUNATOR

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace delaunator
{
constexpr double EPSILON = std::numeric_limits<double>::epsilon();
constexpr double DOUBLE_EPSILON = 2.0 * EPSILON;
constexpr int INVALID_INDEX = -1;

struct Point
{
    double x, y;
    Point(double x, double y) : x(x), y(y){};
    Point() = default;
};

// @see
// https://stackoverflow.com/questions/33333363/built-in-mod-vs-custom-mod-function-improve-the-performance-of-modulus-op/33333636#33333636
inline static int findFastModulus(const int &i, const int &c) noexcept
{
    return i >= c ? i % c : i;
}

// Kahan and Babuska summation, Neumaier variant; accumulates less FP error
inline static double sum(const std::vector<double> &x) noexcept
{
    double sum = x[0];
    double err = 0.0;

    for (int i = 1; i < x.size(); ++i)
    {
        const double k = x[i];
        const double m = sum + k;
        err += std::fabs(sum) >= std::fabs(k) ? sum - m + k : k - m + sum;
        sum = m;
    }
    return sum + err;
}

inline static double dist(const Point &pt_1, const Point &pt_2) noexcept
{
    const double dx = pt_1.x - pt_2.x;
    const double dy = pt_1.y - pt_2.y;
    return dx * dx + dy * dy;
}

inline static double circumradius(const Point &pt_1, const Point &pt_2, const Point &pt_3) noexcept
{
    const double dx = pt_2.x - pt_1.x;
    const double dy = pt_2.y - pt_1.y;
    const double ex = pt_3.x - pt_1.x;
    const double ey = pt_3.y - pt_1.y;
    const double bl = dx * dx + dy * dy;
    const double cl = ex * ex + ey * ey;
    const double d = 0.5 / (dx * ey - dy * ex + EPSILON);
    const double x = (ey * bl - dy * cl) * d;
    const double y = (dx * cl - ex * bl) * d;
    return x * x + y * y;
}

inline static bool orient(const Point &pt_1, const Point &pt_2, const Point &pt_3) noexcept
{
    return (pt_2.y - pt_1.y) * (pt_3.x - pt_2.x) - (pt_2.x - pt_1.x) * (pt_3.y - pt_2.y) < 0.0;
}
inline static std::pair<double, double> circumcenter(const Point &pt_1, const Point &pt_2, const Point &pt_3) noexcept
{
    const double dx = pt_2.x - pt_1.x;
    const double dy = pt_2.y - pt_1.y;
    const double ex = pt_3.x - pt_1.x;
    const double ey = pt_3.y - pt_1.y;
    const double bl = dx * dx + dy * dy;
    const double cl = ex * ex + ey * ey;
    const double d = 0.5 / (dx * ey - dy * ex + EPSILON);
    const double x = pt_1.x + (ey * bl - dy * cl) * d;
    const double y = pt_1.y + (dx * cl - ex * bl) * d;
    return std::make_pair(x, y);
}

inline static bool inCircle(const Point &pt_1, const Point &pt_2, const Point &pt_3, const Point &pt_4) noexcept
{
    const double dx = pt_1.x - pt_4.x;
    const double dy = pt_1.y - pt_4.y;
    const double ex = pt_2.x - pt_4.x;
    const double ey = pt_2.y - pt_4.y;
    const double fx = pt_3.x - pt_4.x;
    const double fy = pt_3.y - pt_4.y;
    const double ap = dx * dx + dy * dy;
    const double bp = ex * ex + ey * ey;
    const double cp = fx * fx + fy * fy;
    return ((dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx)) < 0.0);
}

inline static bool pointsNearlyEqual(const Point &pt_1, const Point &pt_2) noexcept
{
    return (std::fabs(pt_1.x - pt_2.x) < DOUBLE_EPSILON) && (std::fabs(pt_1.y - pt_2.y) < DOUBLE_EPSILON);
}

// monotonically increases with real angle, but doesn't need expensive trigonometry
inline static double findPseudoAngle(const double &dx, const double &dy) noexcept
{
    const double denom = std::abs(dx) + std::abs(dy) + EPSILON;
    const double p = dx / denom;
    return (dy > 0.0 ? 3.0 - p : 1.0 + p) / 4.0; // [0..1)
}

struct Compare
{
    const std::vector<Point> &coords;
    const Point &center;

    Compare(const std::vector<Point> &coords, const Point &center) : coords(coords), center(center){};

    bool operator()(int i, int j) noexcept
    {
        const Point &p_i = coords[i];
        const Point &p_j = coords[j];

        const double d1 = dist(p_i, center);
        const double d2 = dist(p_j, center);

        const double diff1 = d1 - d2;
        const double diff2 = p_i.x - p_j.x;
        const double diff3 = p_i.y - p_j.y;

        // If diff1 equals to zero
        if (diff1 != 0.0)
        {
            return (diff1 < 0);
        }
        // If diff2 equals to zero
        else if (diff2 != 0.0)
        {
            return (diff2 < 0);
        }
        else
        {
            return (diff3 < 0);
        }
    }
};

class Delaunator
{
  public:
    Delaunator(std::vector<Point> const &coords);

    double getHullArea() noexcept;

    const std::vector<Point> &getCoordinates() const
    {
        return coords_;
    }
    const std::vector<int> &getTriangles() const
    {
        return triangles_;
    }
    const std::vector<int> &getHalfEdges() const
    {
        return half_edges_;
    }
    const std::vector<int> &getPreviousHullEdges() const
    {
        return hull_prev_;
    }
    const std::vector<int> &getNextHullEdges() const
    {
        return hull_next_;
    }
    const std::vector<int> &getHullTriangulationEdges() const
    {
        return hull_tri_;
    }
    const std::vector<int> &getHashes() const
    {
        return hash_;
    }
    const std::vector<int> &getEdgeStack() const
    {
        return edge_stack_;
    }
    const Point &getCenterPoint() const
    {
        return center_;
    }
    const int getHashSize() const
    {
        return hash_size_;
    }
    const int getHullStartIndex() const
    {
        return hull_start_;
    }

    void setCenter(const Point &center)
    {
        center_ = center;
    }
    void setCenterX(const double &x)
    {
        center_.x = x;
    }
    void setCenterY(const double &y)
    {
        center_.y = y;
    }

  private:
    const std::vector<Point> &coords_;
    std::vector<int> triangles_;
    std::vector<int> half_edges_;
    std::vector<int> hull_prev_;
    std::vector<int> hull_next_;
    std::vector<int> hull_tri_;
    std::vector<int> hash_;
    std::vector<int> edge_stack_;
    Point center_;
    int hash_size_;
    int hull_start_;

    int legalize(int a);
    int getHashKey(const double &x, const double &y) const noexcept;
    int addTriangle(int i0, int i1, int i2, int a, int b, int c);
    void link(int a, int b);
};

Delaunator::Delaunator(const std::vector<Point> &coords)
    : coords_(coords), triangles_(), half_edges_(), hull_prev_(), hull_next_(), hull_tri_(), hull_start_(), hash_(),
      center_(), hash_size_(), edge_stack_()
{
    int n = coords_.size();

    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    std::vector<int> ids;
    ids.reserve(n);

    for (int i = 0; i < n; ++i)
    {
        const Point &point = coords_[i];
        const double &x = point.x;
        const double &y = point.y;

        if (x < min_x)
        {
            min_x = x;
        }
        if (y < min_y)
        {
            min_y = y;
        }
        if (x > max_x)
        {
            max_x = x;
        }
        if (y > max_y)
        {
            max_y = y;
        }

        ids.push_back(i);
    }

    center_.x = (min_x + max_x) / 2.0;
    center_.y = (min_y + max_y) / 2.0;
    double min_dist = std::numeric_limits<double>::max();

    int i0 = INVALID_INDEX;
    int i1 = INVALID_INDEX;
    int i2 = INVALID_INDEX;

    // pick a seed point close to the centroid
    for (int i = 0; i < n; ++i)
    {
        const double d = dist(center_, coords_[i]);
        if (d < min_dist)
        {
            i0 = i;
            min_dist = d;
        }
    }

    // find the point closest to the seed
    Point point_i0 = coords_[i0];
    min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i)
    {
        if (i == i0)
        {
            continue;
        }
        const double d = dist(point_i0, coords_[i]);
        if (d < min_dist && d > 0.0)
        {
            i1 = i;
            min_dist = d;
        }
    }

    // find the third point which forms the smallest circumcircle with the first two
    Point point_i1 = coords_[i1];
    double min_radius = std::numeric_limits<double>::max();
    for (int i = 0; i < n; ++i)
    {
        if (i == i0 || i == i1)
        {
            continue;
        }

        const double r = circumradius(point_i0, point_i1, coords_[i]);

        if (r < min_radius)
        {
            i2 = i;
            min_radius = r;
        }
    }

    if (!(min_radius < std::numeric_limits<double>::max()))
    {
        throw std::runtime_error("not triangulation");
    }

    Point point_i2 = coords_[i2];

    // swap the order of the seed points for counter-clockwise orientation
    if (orient(point_i0, point_i1, point_i2))
    {
        std::swap(i1, i2);
        std::swap(point_i1, point_i2);
    }

    const auto center = circumcenter(point_i0, point_i1, point_i2);
    setCenterX(center.first);
    setCenterY(center.second);

    // sort the points by distance from the seed triangle circumcenter
    std::sort(ids.begin(), ids.end(), Compare{coords_, center_});

    // initialize a hash table for storing edges of the advancing convex hull
    hash_size_ = static_cast<int>(std::llround(std::ceil(std::sqrt(n))));
    hash_.resize(hash_size_);
    std::fill(hash_.begin(), hash_.end(), INVALID_INDEX);

    // initialize arrays for tracking the edges of the advancing convex hull
    hull_prev_.resize(n);
    hull_next_.resize(n);
    hull_tri_.resize(n);

    hull_start_ = i0;

    int hull_size = 3;

    hull_next_[i0] = hull_prev_[i2] = i1;
    hull_next_[i1] = hull_prev_[i0] = i2;
    hull_next_[i2] = hull_prev_[i1] = i0;

    hull_tri_[i0] = 0;
    hull_tri_[i1] = 1;
    hull_tri_[i2] = 2;

    hash_[getHashKey(point_i0.x, point_i0.y)] = i0;
    hash_[getHashKey(point_i1.x, point_i1.y)] = i1;
    hash_[getHashKey(point_i2.x, point_i2.y)] = i2;

    int max_triangles = n < 3 ? 1 : 2 * n - 5;
    triangles_.reserve(max_triangles * 3);
    half_edges_.reserve(max_triangles * 3);

    addTriangle(i0, i1, i2, INVALID_INDEX, INVALID_INDEX, INVALID_INDEX);

    double xp = std::numeric_limits<double>::quiet_NaN();
    double yp = std::numeric_limits<double>::quiet_NaN();

    Point point_p(xp, yp);
    for (int k = 0; k < n; ++k)
    {
        const int i = ids[k];
        const Point &point_i = coords_[i];

        // skip near-duplicate points
        if (k > 0 && pointsNearlyEqual(point_i, point_p))
        {
            continue;
        }

        point_p.x = point_i.x;
        point_p.y = point_i.y;

        // skip seed triangle points
        if (i == i0 || i == i1 || i == i2)
        {
            continue;
        }

        // find a visible edge on the convex hull using edge hash
        int start = 0;
        int key = getHashKey(point_i.x, point_i.y);
        for (int j = 0; j < hash_size_; ++j)
        {
            start = hash_[findFastModulus(key + j, hash_size_)];
            if (start != INVALID_INDEX && start != hull_next_[start])
            {
                break;
            }
        }

        start = hull_prev_[start];
        int e = start;
        int q;

        while (q = hull_next_[e], !orient(point_i, coords_[e], coords_[q]))
        { // TODO: does it works in a same way as in JS
            e = q;
            if (e == start)
            {
                e = INVALID_INDEX;
                break;
            }
        }

        if (e == INVALID_INDEX)
        {
            continue; // likely a near-duplicate point; skip it
        }

        // add the first triangle from the point
        int t = addTriangle(e, i, hull_next_[e], INVALID_INDEX, INVALID_INDEX, hull_tri_[e]);

        // recursively flip triangles from the point until they satisfy the Delaunay condition
        hull_tri_[i] = legalize(t + 2);

        // keep track of boundary triangles on the hull
        hull_tri_[e] = t;
        ++hull_size;

        // walk forward through the hull, adding more triangles and flipping recursively
        int next = hull_next_[e];
        while (q = hull_next_[next], orient(point_i, coords_[next], coords_[q]))
        {
            t = addTriangle(next, i, q, hull_tri_[i], INVALID_INDEX, hull_tri_[next]);
            hull_tri_[i] = legalize(t + 2);
            hull_next_[next] = next; // mark as removed
            --hull_size;
            next = q;
        }

        // walk backward from the other side, adding more triangles and flipping
        if (e == start)
        {
            while (q = hull_prev_[e], orient(point_i, coords_[q], coords_[e]))
            {
                t = addTriangle(q, i, e, INVALID_INDEX, hull_tri_[e], hull_tri_[q]);
                legalize(t + 2);
                hull_tri_[q] = t;
                hull_next_[e] = e; // mark as removed
                --hull_size;
                e = q;
            }
        }

        // update the hull indices
        hull_prev_[i] = e;
        hull_start_ = e;
        hull_prev_[next] = i;
        hull_next_[e] = i;
        hull_next_[i] = next;

        // save the two new edges in the hash table
        hash_[getHashKey(point_i.x, point_i.y)] = i;
        hash_[getHashKey(coords_[e].x, coords_[e].y)] = e;
    }
}

double Delaunator::getHullArea() noexcept
{
    std::vector<double> hull_area;
    int i = hull_start_;
    do
    {
        hull_area.push_back((coords_[i].x - coords_[hull_prev_[i]].x) * (coords_[i].y + coords_[hull_prev_[i]].y));
        i = hull_next_[i];
    } while (i != hull_start_);
    return sum(hull_area);
}

int Delaunator::legalize(int a)
{
    int i = 0;
    int ar = 0;
    edge_stack_.clear();

    // recursion eliminated with a fixed-size stack
    while (true)
    {
        const int b = half_edges_[a];

        /* if the pair of triangles doesn't satisfy the Delaunay condition
         * (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
         * then do the same check/flip recursively for the new pair of triangles
         *
         *           pl                    pl
         *          /||\                  /  \
         *       al/ || \bl            al/    \a
         *        /  ||  \              /      \
         *       /  a||b  \    flip    /___ar___\
         *     p0\   ||   /p1   =>   p0\---bl---/p1
         *        \  ||  /              \      /
         *       ar\ || /br             b\    /br
         *          \||/                  \  /
         *           pr                    pr
         */
        const int a0 = a - a % 3; // 3 * (a / 3);
        ar = a0 + (a + 2) % 3;

        if (b == INVALID_INDEX)
        { // convex hull edge
            if (i == 0)
            {
                break;
            }
            a = edge_stack_[--i];
            continue;
        }

        const int b0 = b - b % 3;
        const int al = a0 + (a + 1) % 3;
        const int bl = b0 + (b + 2) % 3;

        const int p0 = triangles_[ar];
        const int pr = triangles_[a];
        const int pl = triangles_[al];
        const int p1 = triangles_[bl];

        const bool illegal = inCircle(coords_[p0], coords_[pr], coords_[pl], coords_[p1]);

        if (illegal)
        {
            triangles_[a] = p1;
            triangles_[b] = p0;

            const auto &hbl = half_edges_[bl];

            // edge swapped on the other side of the hull (rare); fix the halfedge reference
            if (hbl == INVALID_INDEX)
            {
                int e = hull_start_;
                do
                {
                    if (hull_tri_[e] == bl)
                    {
                        hull_tri_[e] = a;
                        break;
                    }
                    e = hull_prev_[e];
                } while (e != hull_start_);
            }
            link(a, hbl);
            link(b, half_edges_[ar]);
            link(ar, bl);
            const int br = b0 + (b + 1) % 3;

            if (i < edge_stack_.size())
            {
                edge_stack_[i] = br;
            }
            else
            {
                edge_stack_.push_back(br);
            }
            ++i;
        }
        else
        {
            if (i == 0)
            {
                break;
            }
            a = edge_stack_[--i];
        }
    }
    return ar;
}

inline int Delaunator::getHashKey(const double &x, const double &y) const noexcept
{
    return findFastModulus(static_cast<int>(std::llround(std::floor(findPseudoAngle(x - center_.x, y - center_.y) *
                                                                    static_cast<double>(hash_size_)))),
                           hash_size_);
}

int Delaunator::addTriangle(int i0, int i1, int i2, int a, int b, int c)
{
    int t = triangles_.size();
    triangles_.push_back(i0);
    triangles_.push_back(i1);
    triangles_.push_back(i2);
    link(t, a);
    link(t + 1, b);
    link(t + 2, c);
    return t;
}

void Delaunator::link(const int a, const int b)
{
    int s = half_edges_.size();
    if (a == s)
    {
        half_edges_.push_back(b);
    }
    else if (a < s)
    {
        half_edges_[a] = b;
    }
    else
    {
        throw std::runtime_error("Cannot link edge");
    }
    if (b != INVALID_INDEX)
    {
        int s2 = half_edges_.size();
        if (b == s2)
        {
            half_edges_.push_back(a);
        }
        else if (b < s2)
        {
            half_edges_[b] = a;
        }
        else
        {
            throw std::runtime_error("Cannot link edge");
        }
    }
}

} // namespace delaunator

#endif // DELAUNATOR