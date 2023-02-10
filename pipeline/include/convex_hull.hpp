#ifndef CONVEX_HULL
#define CONVEX_HULL

#include "internal_types.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

// Helper methods
template <typename Floating, std::enable_if_t<std::is_floating_point<Floating>::value, int> = 0>
inline bool areEqual(const Floating &value_1, const Floating &value_2) noexcept
{
    return std::fabs(value_1 - value_2) < std::numeric_limits<Floating>::epsilon();
}

template <typename Integer, std::enable_if_t<std::is_integral<Integer>::value, int> = 0>
inline bool areEqual(const Integer &value_1, const Integer &value_2) noexcept
{
    return (value_1 == value_2);
}

template <typename T> inline T getArea(const Point<T> &a, const Point<T> &b, const Point<T> &c) noexcept
{
    return (((b).x - (a).x) * ((c).y - (a).y) - ((b).y - (a).y) * ((c).x - (a).x));
}

template <typename T> inline bool rightTurn(const Point<T> &a, const Point<T> &b, const Point<T> &c) noexcept
{
    return getArea(a, b, c) < 0;
}

template <typename T> inline bool leftTurn(const Point<T> &a, const Point<T> &b, const Point<T> &c) noexcept
{
    return getArea(a, b, c) > 0;
}

template <typename T, std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
inline bool collinear(const Point<T> &a, const Point<T> &b, const Point<T> &c) noexcept
{
    return std::fabs(getArea(a, b, c)) <= std::numeric_limits<T>::epsilon();
}

template <typename T, std::enable_if_t<std::is_integral<T>::value, int> = 0>
inline bool collinear(const Point<T> &a, const Point<T> &b, const Point<T> &c) noexcept
{
    return getArea(a, b, c) == 0;
}

template <typename T> inline std::int8_t sign(const T x) noexcept
{
    return (x < 0) ? -1 : ((x > 0) ? 1 : 0);
}

template <typename T> inline std::int8_t cmp(const Point<T> &a, const Point<T> &b) noexcept
{
    return (areEqual(a.x, b.x) ? sign(a.y - b.y) : sign(a.x - b.x));
}

template <typename T> inline bool comparePoints(const Point<T> &a, const Point<T> &b) noexcept
{
    return (areEqual(a.x, b.x) && areEqual(a.y, b.y));
}

// Convex hull
template <typename T> class ConvexHullGenerator
{
  private:
    // Input points
    std::vector<Point<T>> points_;

    // Quadrant Boundary Points
    std::unique_ptr<Quadrant<T>> q1_;
    std::unique_ptr<Quadrant<T>> q2_;
    std::unique_ptr<Quadrant<T>> q3_;
    std::unique_ptr<Quadrant<T>> q4_;

    // Quadrant Points
    std::deque<Point<T>> q1_pts_;
    std::deque<Point<T>> q2_pts_;
    std::deque<Point<T>> q3_pts_;
    std::deque<Point<T>> q4_pts_;

    // Quadrant Point Count
    std::size_t q1_count_ = 0;
    std::size_t q2_count_ = 0;
    std::size_t q3_count_ = 0;
    std::size_t q4_count_ = 0;

    // Convex Hull
    bool should_close_the_graph_;
    std::vector<Point<T>> convex_hull_points_;

  public:
    explicit ConvexHullGenerator(const std::vector<Point<T>> &points, bool should_close_the_graph = true)
        : points_(points), should_close_the_graph_(should_close_the_graph)
    {
        if (!points_.empty())
        {
            calculateConvexHull();
        }
    };

    explicit ConvexHullGenerator(bool should_close_the_graph = true)
        : should_close_the_graph_(should_close_the_graph){};

    ~ConvexHullGenerator() = default;

    void calculateConvexHull(const std::vector<Point<T>> &points, std::vector<Point<T>> &hull_points)
    {
        hull_points.clear();
        if (points.empty())
        {
            return;
        }
        points_.clear();
        points_.reserve(points.size());
        std::copy(points.begin(), points.end(), std::back_insert_iterator(points_));
        convex_hull_points_.clear();
        calculateConvexHull();
        std::copy(convex_hull_points_.begin(), convex_hull_points_.end(), std::back_insert_iterator(hull_points));
    }

    // Caution, return by const ref!
    const std::vector<Point<T>> &getConvexHull() const
    {
        return convex_hull_points_;
    }

    std::tuple<Quadrant<T>, Quadrant<T>, Quadrant<T>, Quadrant<T>> getQuadrantPoints() const
    {
        return std::make_tuple(*q1_, *q2_, *q3_, *q4_);
    }

  private:
    void calculateConvexHull()
    {
        // See: https://www.codeproject.com/Articles/775753/A-Convex-Hull-Algorithm-and-its-implementation-in

        // Find Quadrant Limits
        setQuadrantLimits();

        // Accumulate Hull Points
        accumulateHullPoints();

        // Combine Hull Points
        combineHullPoints();
    }

    void setQuadrantLimits()
    {
        const Point<T> &first_point = points_[0];
        const auto x_first_point = first_point.x;
        const auto y_first_point = first_point.y;

        // Each quadrant will have two points - start, end and root
        // By convention, start to end motion in counterclockwise
        //
        // Quadrant: Q2 | Q1
        //	         -------
        //           Q3 | Q4
        //
        // * Denote start and end points in each quadrant
        // R Denote root points
        //
        //      /|
        //     / |\
        //    /  * \
        //   /   |  \  
        //  /    R_*_R___*____
        // /     |   |       /
        // __*___R___R      /
        // \         |     /
        //  \        |    /
        //   \       *   /
        //    \      |  /
        //     \     | /
        //      \    |/
        //       \   |
        //        \  |
        //         \ |
        //          \|

        Point<T> q1_start = first_point, q1_end = first_point, q1_root = first_point;
        Point<T> q2_start = first_point, q2_end = first_point, q2_root = first_point;
        Point<T> q3_start = first_point, q3_end = first_point, q3_root = first_point;
        Point<T> q4_start = first_point, q4_end = first_point, q4_root = first_point;

        for (const Point<T> &point : points_)
        {
            // top right
            if (point.x > q1_start.x)
            {
                q1_start = point;
                q4_end = point;
            }
            else if (areEqual(point.x, q1_start.x))
            {
                if (point.y > q1_start.y)
                {
                    q1_start = point;
                }
                else if (point.y < q4_end.y)
                {
                    q4_end = point;
                }
            }

            // bottom left
            if (point.x < q2_end.x)
            {
                q2_end = point;
                q3_start = point;
            }
            else if (areEqual(point.x, q2_end.x))
            {
                if (point.y > q2_end.y)
                {
                    q2_end = point;
                }
                else if (point.y < q3_start.y)
                {
                    q3_start = point;
                }
            }

            // top left
            if (point.y > q1_end.y)
            {
                q1_end = point;
                q2_start = point;
            }
            else if (areEqual(point.y, q1_end.y))
            {
                if (point.x < q2_start.x)
                {
                    q2_start = point;
                }
                else if (point.x > q1_end.x)
                {
                    q1_end = point;
                }
            }

            // bottom right
            if (point.y < q3_end.y)
            {
                q3_end = point;
                q4_start = point;
            }
            else if (areEqual(point.y, q3_end.y))
            {
                if (point.x < q3_end.x)
                {
                    q3_end = point;
                }
                else if (point.x > q4_start.x)
                {
                    q4_start = point;
                }
            }
        }

        // Set root points
        q1_root.x = q1_end.x;
        q1_root.y = q1_start.y;

        q2_root.x = q2_start.x;
        q2_root.y = q2_end.y;

        q3_root.x = q3_end.x;
        q3_root.y = q3_start.y;

        q4_root.x = q4_start.x;
        q4_root.y = q4_end.y;

        // Set quadrant limits
        q1_ = std::make_unique<Quadrant<T>>(q1_start, q1_end, q1_root);
        q2_ = std::make_unique<Quadrant<T>>(q2_start, q2_end, q2_root);
        q3_ = std::make_unique<Quadrant<T>>(q3_start, q3_end, q3_root);
        q4_ = std::make_unique<Quadrant<T>>(q4_start, q4_end, q4_root);

        // Initialize quadrants
        auto initializeQuadrant = [&](std::deque<Point<T>> &points, std::size_t &hull_count, const Point<T> &point_1,
                                      const Point<T> &point_2) -> void {
            points.push_back(point_1);
            if (comparePoints(point_1, point_2))
            {
                hull_count = 1;
            }
            else
            {
                points.push_back(point_2);
                hull_count = 2;
            }
        };

        initializeQuadrant(q1_pts_, q1_count_, q1_start, q1_end);
        initializeQuadrant(q2_pts_, q2_count_, q2_start, q2_end);
        initializeQuadrant(q3_pts_, q3_count_, q3_start, q3_end);
        initializeQuadrant(q4_pts_, q4_count_, q4_start, q4_end);
    }

    // Insert point into convex hull
    inline void insertPoint(std::deque<Point<T>> &points, const std::size_t index, const Point<T> &point,
                            std::size_t &count)
    {
        points.insert(points.begin() + index, point);
        ++count;
    }

    // Remove every item from index start to index end inclusive
    inline void removeRange(std::deque<Point<T>> &points, const std::size_t index_start, const std::size_t index_end,
                            std::size_t &count)
    {
        points.erase(points.begin() + index_start, points.begin() + index_end + 1);
        count -= (index_end - index_start + 1);
    }

    inline void updatePoints(std::size_t &index_low, std::size_t &index_high, std::deque<Point<T>> &points,
                             const Point<T> &point, std::size_t &count)
    {
        while (index_low > 0)
        {
            if (rightTurn(points[index_low - 1], point, points[index_low]))
            {
                break;
            }
            --index_low;
        }
        std::size_t max_index_high = count - 1;
        while (index_high < max_index_high)
        {
            if (rightTurn(point, points[index_high + 1], points[index_high]))
            {
                break;
            }
            ++index_high;
        }
        if (index_low + 1 == index_high)
        {
            insertPoint(points, index_low + 1, point, count);
        }
        else if (index_low + 2 == index_high)
        {
            points[index_low + 1] = point;
        }
        else
        {
            points[index_low + 1] = point;
            removeRange(points, index_low + 2, index_high - 1, count);
        }
    }

    inline bool checkIfBelongsToFirstQuadrant(const Point<T> &point)
    {
        if (point.x > q1_->root_point.x && point.y > q1_->root_point.y)
        {
            std::size_t index_low = 0;
            std::size_t index_high = q1_count_;
            while (index_low < index_high - 1)
            {
                std::size_t index = ((index_high - index_low) >> 1) + index_low;
                if (point.x <= q1_pts_[index].x && point.y <= q1_pts_[index].y)
                {
                    return false;
                }
                if (point.x > q1_pts_[index].x)
                {
                    index_high = index;
                    continue;
                }
                if (point.x < q1_pts_[index].x)
                {
                    index_low = index;
                    continue;
                }
                index_low = index - 1;
                index_high = index + 1;
                break;
            }
            if (!rightTurn(q1_pts_[index_low], q1_pts_[index_high], point))
            {
                return false;
            }
            updatePoints(index_low, index_high, q1_pts_, point, q1_count_);
            return true;
        }
        return false;
    }

    inline bool checkIfBelongsToSecondQuadrant(const Point<T> &point)
    {
        if (point.x < q2_->root_point.x && point.y > q2_->root_point.y)
        {
            std::size_t index_low = 0;
            std::size_t index_high = q2_count_;
            while (index_low < index_high - 1)
            {
                std::size_t index = ((index_high - index_low) >> 1) + index_low;
                if (point.x >= q2_pts_[index].x && point.y <= q2_pts_[index].y)
                {
                    return false;
                }
                if (point.x > q2_pts_[index].x)
                {
                    index_high = index;
                    continue;
                }
                if (point.x < q2_pts_[index].x)
                {
                    index_low = index;
                    continue;
                }
                index_low = index - 1;
                index_high = index + 1;
                break;
            }
            if (!rightTurn(q2_pts_[index_low], q2_pts_[index_high], point))
            {
                return false;
            }
            updatePoints(index_low, index_high, q2_pts_, point, q2_count_);
            return true;
        }
        return false;
    }

    inline bool checkIfBelongsToThirdQuadrant(const Point<T> &point)
    {
        if (point.x < q3_->root_point.x && point.y < q3_->root_point.y)
        {
            std::size_t index_low = 0;
            std::size_t index_high = q3_count_;

            while (index_low < index_high - 1)
            {
                std::size_t index = ((index_high - index_low) >> 1) + index_low;
                if (point.x >= q3_pts_[index].x && point.y >= q3_pts_[index].y)
                {
                    return false;
                }
                if (point.x < q3_pts_[index].x)
                {
                    index_high = index;
                    continue;
                }
                if (point.x > q3_pts_[index].x)
                {
                    index_low = index;
                    continue;
                }
                index_low = index - 1;
                index_high = index + 1;
                break;
            }
            if (!rightTurn(q3_pts_[index_low], q3_pts_[index_high], point))
            {
                return false;
            }
            updatePoints(index_low, index_high, q3_pts_, point, q3_count_);
            return true;
        }
        return false;
    }

    inline bool checkIfBelongsToFourthQuadrant(const Point<T> &point)
    {
        if (point.x > q4_->root_point.x && point.y < q4_->root_point.y)
        {
            std::size_t index_low = 0;
            std::size_t index_high = q4_count_;
            while (index_low < index_high - 1)
            {
                std::size_t index = ((index_high - index_low) >> 1) + index_low;
                if (point.x <= q4_pts_[index].x && point.y >= q4_pts_[index].y)
                {
                    return false;
                }
                if (point.x < q4_pts_[index].x)
                {
                    index_high = index;
                    continue;
                }
                if (point.x > q4_pts_[index].x)
                {
                    index_low = index;
                    continue;
                }
                index_low = index - 1;
                index_high = index + 1;
                break;
            }
            if (!rightTurn(q4_pts_[index_low], q4_pts_[index_high], point))
            {
                return false;
            }
            updatePoints(index_low, index_high, q4_pts_, point, q4_count_);
            return true;
        }
        return false;
    }

    void accumulateHullPoints()
    {
        for (const Point<T> &point : points_)
        {
            if (checkIfBelongsToFirstQuadrant(point))
            {
                continue;
            }
            else if (checkIfBelongsToSecondQuadrant(point))
            {
                continue;
            }
            else if (checkIfBelongsToThirdQuadrant(point))
            {
                continue;
            }
            else if (checkIfBelongsToFourthQuadrant(point))
            {
                continue;
            }
        }
    }

    void combineHullPoints()
    {
        std::size_t index_q1_start = 0;
        std::size_t index_q1_end = q1_count_ - 1;
        std::size_t index_q2_start, index_q2_end;
        std::size_t index_q3_start, index_q3_end;
        std::size_t index_q4_start, index_q4_end;
        Point<T> point_last = q1_pts_[index_q1_end];

        auto setPoints = [&point_last](const std::size_t count, std::size_t &idx_start, std::size_t &idx_end,
                                       const std::deque<Point<T>> &points) -> void {
            if (count == 1)
            {
                if (comparePoints(points[0], point_last))
                {
                    idx_start = 1;
                    idx_end = 0;
                }
                else
                {
                    idx_start = 0;
                    idx_end = 0;
                    point_last = points[0];
                }
            }
            else
            {
                if (comparePoints(points[0], point_last))
                {
                    idx_start = 1;
                }
                else
                {
                    idx_start = 0;
                }
                idx_end = count - 1;
                point_last = points[idx_end];
            }
        };

        setPoints(q2_count_, index_q2_start, index_q2_end, q2_pts_);
        setPoints(q3_count_, index_q3_start, index_q3_end, q3_pts_);
        setPoints(q4_count_, index_q4_start, index_q4_end, q4_pts_);

        if (comparePoints(q1_pts_[index_q1_start], point_last))
        {
            ++index_q1_start;
        }

        std::size_t count_of_final_hull_point =
            (index_q1_end - index_q1_start + 1) + (index_q2_end - index_q2_start + 1) +
            (index_q3_end - index_q3_start + 1) + (index_q4_end - index_q4_start + 1);

        if (count_of_final_hull_point <= 1)
        {
            convex_hull_points_ = {point_last};
            return;
        }

        if (count_of_final_hull_point > 1 && should_close_the_graph_)
        {
            ++count_of_final_hull_point;
        }

        convex_hull_points_.reserve(count_of_final_hull_point);

        for (std::size_t i = index_q1_start; i < index_q1_end + 1; ++i)
        {
            convex_hull_points_.emplace_back(std::move(q1_pts_[i]));
        }

        for (std::size_t i = index_q2_start; i < index_q2_end + 1; ++i)
        {
            convex_hull_points_.emplace_back(std::move(q2_pts_[i]));
        }

        for (std::size_t i = index_q3_start; i < index_q3_end + 1; ++i)
        {
            convex_hull_points_.emplace_back(std::move(q3_pts_[i]));
        }

        for (std::size_t i = index_q4_start; i < index_q4_end + 1; ++i)
        {
            convex_hull_points_.emplace_back(std::move(q4_pts_[i]));
        }

        if (count_of_final_hull_point > 1 && should_close_the_graph_)
        {
            convex_hull_points_.push_back(convex_hull_points_[0]);
        }
    }
};

#endif // CONVEX_HULL