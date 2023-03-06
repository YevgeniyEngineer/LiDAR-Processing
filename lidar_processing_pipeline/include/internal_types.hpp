#ifndef INTERNAL_TYPES
#define INTERNAL_TYPES

#include <cmath>
#include <limits>
#include <vector>

namespace lidar_processing
{
struct PointXY final
{
    // Explicit constructor
    explicit PointXY(float x, float y) : x(x), y(y){};

    // Default constructor
    PointXY() = default;

    // Default destructor
    ~PointXY() noexcept = default;

    // Copy constructor
    PointXY(const PointXY &other) = default;

    // Copy assignment operator
    PointXY &operator=(const PointXY &rhs) = default;

    // Move constructor
    PointXY(PointXY &&) noexcept = default;

    // Move assignment operator
    PointXY &operator=(PointXY &&other) noexcept = default;

    float x;
    float y;
};

// Operator that returns result of comparison of two points
// without taking index into consideration
inline bool operator==(const PointXY &lhs, const PointXY &rhs)
{
    return (lhs.x == rhs.x && lhs.y == rhs.y);
}

/// @brief Lexicographic comparison of two points
/// @param lhs Point 1
/// @param rhs Point 2
/// @return True if either x(lhs) < x(rhs) or y(lhs) < y(rhs)
inline static bool operator<(const PointXY &lhs, const PointXY &rhs) noexcept
{
    return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
}

/// @brief Euclidean distance squared between two points
/// @param lhs Point 1
/// @param rhs Point 2
/// @return Distance squared
inline static float distanceSquared(const PointXY &lhs, const PointXY &rhs) noexcept
{
    float dx = lhs.x - rhs.x;
    float dy = lhs.y - rhs.y;
    return (dx * dx) + (dy * dy);
}

/// @brief Euclidean distance between two points
/// @param lhs Point 1
/// @param rhs Point 2
/// @return Distance
inline static float distance(const PointXY &lhs, const PointXY &rhs) noexcept
{
    return std::sqrt(distanceSquared(lhs, rhs));
}

struct PointXYZ final
{
    // Explicit constructor
    explicit PointXYZ(float x, float y, float z) : x(x), y(y), z(z){};

    // Default constructor
    PointXYZ() = default;

    // Default destructor
    ~PointXYZ() noexcept = default;

    // Copy constructor
    PointXYZ(const PointXYZ &other) = default;

    // Copy assignment operator
    PointXYZ &operator=(const PointXYZ &rhs) = default;

    // Move constructor
    PointXYZ(PointXYZ &&) noexcept = default;

    // Move assignment operator
    PointXYZ &operator=(PointXYZ &&other) noexcept = default;

    float x;
    float y;
    float z;
};

// Operator that returns result of comparison of two points
// without taking index into consideration
inline static bool operator==(const PointXYZ &lhs, const PointXYZ &rhs) noexcept
{
    return (lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z);
}

/// @brief Euclidean distance squared between two points
/// @param lhs Point 1
/// @param rhs Point 2
/// @return Distance squared
inline static float distanceSquared(const PointXYZ &lhs, const PointXYZ &rhs) noexcept
{
    float dx = lhs.x - rhs.x;
    float dy = lhs.y - rhs.y;
    float dz = lhs.z - rhs.z;
    return (dx * dx) + (dy * dy) + (dz * dz);
}

/// @brief Euclidean distance between two points
/// @param lhs Point 1
/// @param rhs Point 2
/// @return Distance
inline static float distance(const PointXYZ &lhs, const PointXYZ &rhs) noexcept
{
    return std::sqrt(distanceSquared(lhs, rhs));
}

// A vertex in 2D space
struct Vec2
{
    float x, y;
    Vec2() : x(0), y(0){};
    constexpr Vec2(float x, float y) : x(x), y(y){};

    Vec2 operator+(const Vec2 &rhs) const noexcept
    {
        return Vec2(x + rhs.x, y + rhs.y);
    }

    Vec2 operator-(const Vec2 &rhs) const noexcept
    {
        return Vec2(x - rhs.x, y - rhs.y);
    }

    Vec2 operator+(float num) const noexcept
    {
        return Vec2(x + num, y + num);
    }

    Vec2 operator-(float num) const noexcept
    {
        return Vec2(x - num, y - num);
    }

    Vec2 operator*(float num) const noexcept
    {
        return Vec2(x * num, y * num);
    }

    float dot(const Vec2 &rhs) const noexcept
    {
        return x * rhs.x + y * rhs.y;
    }

    float cross(const Vec2 &rhs) const noexcept
    {
        return x * rhs.y - y * rhs.x;
    }

    constexpr float mag2() const noexcept
    {
        return x * x + y * y;
    }

    constexpr float mag() const noexcept
    {
        return std::sqrt(mag2());
    }

    constexpr Vec2 normalized() const noexcept
    {
        float mag = std::sqrt(mag2());
        if (mag < std::numeric_limits<float>::epsilon())
        {
            return Vec2(0, 0);
        }
        else
        {
            return Vec2(x / mag, y / mag);
        }
    }

    constexpr Vec2 perp() const noexcept
    {
        return Vec2(-y, x);
    }
};

// A line segment in 2D space
struct Segment2
{
    Vec2 a, b;

    Segment2(){};
    constexpr Segment2(const Vec2 &a, const Vec2 &b) : a(a), b(b){};

    constexpr Vec2 dir() const noexcept
    {
        return b - a;
    }

    constexpr Vec2 normal() const noexcept
    {
        return dir().perp().normalized();
    }

    float length() const noexcept
    {
        return dir().mag();
    }

    Vec2 closest_point(const Vec2 &p) const noexcept
    {
        Vec2 d = dir().normalized();
        float t = (p - a).dot(d);
        t = std::max(0.0f, std::min(t, length()));
        return a + d * t;
    }
};

} // namespace lidar_processing

#endif // INTERNAL_TYPES