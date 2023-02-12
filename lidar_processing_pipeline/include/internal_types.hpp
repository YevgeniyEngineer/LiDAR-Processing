#ifndef INTERNAL_TYPES
#define INTERNAL_TYPES

#include <cstdint>

template <typename T> struct Point
{
    // Explicit constructor
    explicit Point(const T x, const T y, const std::size_t index) : x(x), y(y), index(index){};

    // Default constructor
    Point() = default;

    // Default destructor
    virtual ~Point() = default;

    // Copy constructor
    Point(const Point &other) = default;

    // Copy assignment operator
    Point &operator=(const Point &rhs) = default;

    // Move constructor
    Point(Point &&) = default;

    // Move assignment operator
    Point &operator=(Point &&other) = default;

    // Operator that returns result of comparison of two points
    // without taking index into consideration
    friend bool operator==(const Point<T> &lhs, const Point<T> &rhs)
    {
        return (lhs.x == rhs.x && lhs.y == rhs.y);
    }

    T x;
    T y;
    std::size_t index;
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