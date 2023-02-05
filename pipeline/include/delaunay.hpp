#ifndef DELAUNAY
#define DELAUNAY

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace lidar_processing
{
// Point in 2D
struct Point
{
    float x, y;
};

// Edge consists of two points
struct Edge
{
    Point p1, p2;
};

// Triangle consists of three points
struct Triangle
{
    Point p1, p2, p3;
};

class DelaunayTriangulation
{
  public:
    DelaunayTriangulation() = default;
    ~DelaunayTriangulation() = default;

  private:
    // monotonically increases with real angle, but doesn't need expensive trigonometry
    float pseudoAngle(const float &dx, const float &dy) const noexcept
    {
        const float p = dx / (std::fabs(dx) + std::fabs(dy) + std::numeric_limits<float>::epsilon());
        return (dy > 0.0f ? 3.0f - p : 1.0f + p) / 4.0f; // [0..1]
    }

    float dist(const float &ax, const float &ay, const float &bx, const float &by) const noexcept
    {
        const float dx = ax - bx;
        const float dy = ay - by;
        return dx * dx + dy * dy;
    }

    bool inCircle(const float &ax, const float &ay, const float &bx, const float &by, const float &cx, const float &cy,
                  const float &px, const float &py) const noexcept
    {
        const float dx = ax - px;
        const float dy = ay - py;
        const float ex = bx - px;
        const float ey = by - py;
        const float fx = cx - px;
        const float fy = cy - py;

        const float ap = dx * dx + dy * dy;
        const float bp = ex * ex + ey * ey;
        const float cp = fx * fx + fy * fy;

        return ((dx * (ey * cp - bp * fy) - dy * (ex * cp - bp * fx) + ap * (ex * fy - ey * fx)) < 0);
    }

    float circumradius(const float &ax, const float &ay, const float &bx, const float &by, const float &cx,
                       const float &cy) const noexcept
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float ex = cx - ax;
        const float ey = cy - ay;

        const float bl = dx * dx + dy * dy;
        const float cl = ex * ex + ey * ey;
        const float denom = (dx * ey - dy * ex) + std::numeric_limits<float>::epsilon();

        const float d = 0.5 / denom;

        const float x = (ey * bl - dy * cl) * d;
        const float y = (dx * cl - ex * bl) * d;

        return x * x + y * y;
    }

    Point circumcenter(const float &ax, const float &ay, const float &bx, const float &by, const float &cx,
                       const float &cy) const noexcept
    {
        const float dx = bx - ax;
        const float dy = by - ay;
        const float ex = cx - ax;
        const float ey = cy - ay;

        const float bl = dx * dx + dy * dy;
        const float cl = ex * ex + ey * ey;
        const float denom = (dx * ey - dy * ex) + std::numeric_limits<float>::epsilon();

        const float d = 0.5 / denom;

        const float x = ax + (ey * bl - dy * cl) * d;
        const float y = ay + (dx * cl - ex * bl) * d;

        return Point{x, y};
    }
};
} // namespace lidar_processing

#endif // DELAUNAY