#ifndef INTERNAL_TYPES
#define INTERNAL_TYPES

#include <cstdint>

template <typename T> struct Point
{
    explicit Point(T x, T y, const std::size_t &index) : x(x), y(y), index(index){};

    ~Point() = default;

    friend bool operator==(const Point<T> &lhs, const Point<T> &rhs)
    {
        return (lhs.x == rhs.x && lhs.y == rhs.y);
    }

    T x = 0;
    T y = 0;
    std::size_t index = 0;
};

template <typename T> struct Quadrant
{
    explicit Quadrant(const Point<T> &first_point, const Point<T> &last_point, const Point<T> &root_point)
        : first_point(first_point), last_point(last_point), root_point(root_point){};

    ~Quadrant() = default;

    Point<T> first_point;
    Point<T> last_point;
    Point<T> root_point;
};

#endif // INTERNAL_TYPES