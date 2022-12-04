#pragma once
#ifndef CONVEX_HULL_HPP_
#define CONVEX_HULL_HPP_

#include <deque>
#include <iostream>
#include <memory>
#include <vector>

// Left-turn, right-turn and collinear predicates
#define area(a, b, c) (((b).x - (a).x) * ((c).y - (a).y) - ((b).y - (a).y) * ((c).x - (a).x))
#define rightTurn(a, b, c) (area(a, b, c) < 0)
#define leftTurn(a, b, c) (area(a, b, c) > 0)
#define collinear(a, b, c) (area(a, b, c) == 0)

// Macros for lexicographic comparison of two points
#define sign(x) (((x) < 0) ? -1 : (((x) > 0) ? 1 : 0))
#define cmp(a, b) (((a).x == (b).x) ? sign((a).y - (b).y) : sign((a).x - (b).x))
#define comparePoints(a, b) (((a).x == (b).x) && ((a).y == (b).y))

namespace lidar_processing
{
typedef struct
{
    float x, y;
} point_t;

class ConvexHull
{
  public:
    ConvexHull(std::deque<point_t> points, bool should_close_the_graph = true);
    ~ConvexHull() = default;
    std::deque<point_t> getResultAsArray(int &count);

  private:
    static const int quadrant_hull_point_array_initial_capacity_ = 1000;
    static const int quadrant_hull_point_array_grow_size_ = 1000;

    std::deque<point_t> points_;
    int count_of_point_;
    bool should_close_the_graph_;

    std::deque<point_t> q1p_hull_points_;
    std::deque<point_t> q1p_hull_last_;
    int q1_hull_capacity_;
    int q1_hull_count_ = 0;

    std::deque<point_t> q2p_hull_points_;
    std::deque<point_t> q2p_hull_last_;
    int q2_hull_capacity_;
    int q2_hull_count_ = 0;

    std::deque<point_t> q3p_hull_points_;
    std::deque<point_t> q3p_hull_last_;
    int q3_hull_capacity_;
    int q3_hull_count_ = 0;

    std::deque<point_t> q4p_hull_points_;
    std::deque<point_t> q4p_hull_last_;
    int q4_hull_capacity_;
    int q4_hull_count_ = 0;

    void calculateConvexHull();

    inline static void insertPoint(std::deque<point_t> &p_point, int index, point_t &pt, int &count);
    inline static void removeRange(std::deque<point_t> &p_point, int index_start, int index_end, int &count);
};
} // namespace lidar_processing

#endif // CONVEX_HULL_HPP_