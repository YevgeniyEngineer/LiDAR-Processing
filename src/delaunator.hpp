#ifndef LIDAR_PROCESSING__DELAUNATOR_HPP
#define LIDAR_PROCESSING__DELAUNATOR_HPP

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace lidar_processing
{
struct PointXY final
{
    double x{0.0F};
    double y{0.0F};

    PointXY() noexcept = default;

    PointXY(double x, double y) noexcept : x(x), y(y)
    {
    }

    inline double magnitude_squared() const noexcept
    {
        return (x * x) + (y * y);
    }

    inline double magnitude() const noexcept
    {
        return std::sqrt(magnitude_squared());
    }

    static double determinant(const PointXY &p1, const PointXY &p2) noexcept
    {
        return (p1.x * p2.y) - (p1.y * p2.x);
    }

    static PointXY vector(const PointXY &v1, const PointXY &v2) noexcept
    {
        return {v2.x - v1.x, v2.y - v1.y};
    }

    static double distance_squared(const PointXY &v1, const PointXY &v2) noexcept
    {
        const auto vec = vector(v1, v2);
        return (vec.x * vec.x) + (vec.y * vec.y);
    }

    static bool equal(const PointXY &p1, const PointXY &p2, double span) noexcept
    {
        const double dist = distance_squared(p1, p2) / span;
        return (dist < std::numeric_limits<double>::epsilon());
    }
};

constexpr inline std::size_t fast_modulus(std::size_t i, std::size_t c) noexcept
{
    return (i >= c) ? (i % c) : i;
}

constexpr inline double sum(const std::vector<double> &x) noexcept
{
    double sum = 0.0;
    double err = 0.0;

    for (std::size_t i = 1; i < x.size(); ++i)
    {
        const double k = x[i];
        const double m = sum + k;

        err += (std::fabs(sum) >= std::fabs(k)) ? (sum - m + k) : (k - m + sum);
        sum = m;
    }

    return sum + err;
}

constexpr inline double distance_squared(const double ax, const double ay, const double bx, const double by) noexcept
{
    const double dx = ax - bx;
    const double dy = ay - by;

    return dx * dx + dy * dy;
}

// constexpr inline double circumradius(const Point)

constexpr inline std::size_t next_half_edge(std::size_t e) noexcept
{
    return (e % 3 == 2) ? (e - 2) : (e + 1);
}

constexpr inline std::size_t prev_half_edge(std::size_t e) noexcept
{
    return (e % 3 == 0) ? (e + 2) : (e - 1);
}

class Delaunator final
{
  public:
  private:
};

} // namespace lidar_processing

#endif // LIDAR_PROCESSING__DELAUNATOR_HPP
