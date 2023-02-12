#include "convex_hull.hpp"

namespace lidar_processing
{
// constructor
ConvexHull::ConvexHull(const std::vector<point_t> &points, bool should_close_the_graph)
    : points_(points), count_of_point_(points.size()), should_close_the_graph_(should_close_the_graph)
{
    this->calculateConvexHull();
}

// construct convex hull
void ConvexHull::calculateConvexHull()
{
    if (points_.empty())
    {
        std::cout << "Warning: no points were provided to calculateConvexHull" << std::endl;
        return;
    }

    // find quadrant limits (maximum x and y)
    auto points_iterator = points_.begin();
    point_t q1_p1 = *points_iterator;
    point_t q1_p2 = *points_iterator;
    point_t q2_p1 = *points_iterator;
    point_t q2_p2 = *points_iterator;
    point_t q3_p1 = *points_iterator;
    point_t q3_p2 = *points_iterator;
    point_t q4_p1 = *points_iterator;
    point_t q4_p2 = *points_iterator;

    ++points_iterator;

    for (int n = count_of_point_ - 1; n > 0; --n) // -1 because of 0 bound
    {
        point_t &pt = *points_iterator;

        // right
        if (pt.x >= q1_p1.x)
        {
            if (pt.x == q1_p1.x)
            {
                if (pt.y > q1_p1.y)
                {
                    q1_p1 = pt;
                }
                else
                {
                    if (pt.y < q4_p2.y)
                    {
                        q4_p2 = pt;
                    }
                }
            }
            else
            {
                q1_p1 = pt;
                q4_p2 = pt;
            }
        }

        // left
        if (pt.x <= q2_p2.x)
        {
            if (pt.x == q2_p2.x)
            {
                if (pt.y > q2_p2.y)
                {
                    q2_p2 = pt;
                }
                else
                {
                    if (pt.y < q3_p1.y)
                    {
                        q3_p1 = pt;
                    }
                }
            }
            else
            {
                q2_p2 = pt;
                q3_p1 = pt;
            }
        }

        // top
        if (pt.y >= q1_p2.y)
        {
            if (pt.y == q1_p2.y)
            {
                if (pt.x < q2_p1.x)
                {
                    q2_p1 = pt;
                }
                else
                {
                    if (pt.x > q1_p2.x)
                    {
                        q1_p2 = pt;
                    }
                }
            }
            else
            {
                q1_p2 = pt;
                q2_p1 = pt;
            }
        }

        // bottom
        if (pt.y <= q3_p2.y)
        {
            if (pt.y == q3_p2.y)
            {
                if (pt.x < q3_p2.x)
                {
                    q3_p2 = pt;
                }
                else
                {
                    if (pt.x > q4_p1.x)
                    {
                        q4_p1 = pt;
                    }
                }
            }
            else
            {
                q3_p2 = pt;
                q4_p1 = pt;
            }
        }

        ++points_iterator;
    }

    point_t q1_root_pt;
    q1_root_pt.x = q1_p2.x;
    q1_root_pt.y = q1_p1.y;

    point_t q2_root_pt;
    q2_root_pt.x = q2_p1.x;
    q2_root_pt.y = q2_p2.y;

    point_t q3_root_pt;
    q3_root_pt.x = q3_p2.x;
    q3_root_pt.y = q3_p1.y;

    point_t q4_root_pt;
    q4_root_pt.x = q4_p1.x;
    q4_root_pt.y = q4_p2.y;

    auto initializeQuadrant = [&](std::deque<point_t> &points, int &hull_count, const point_t &point_1,
                                  const point_t &point_2) -> void {
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

    // initialize quadrants
    initializeQuadrant(q1p_hull_points_, q1_hull_count_, q1_p1, q1_p2);
    initializeQuadrant(q2p_hull_points_, q2_hull_count_, q2_p1, q2_p2);
    initializeQuadrant(q3p_hull_points_, q3_hull_count_, q3_p1, q3_p2);
    initializeQuadrant(q4p_hull_points_, q4_hull_count_, q4_p1, q4_p2);

    // start calculation
    int index;
    int index_low;
    int index_high;

    points_iterator = points_.begin();
    for (int n = count_of_point_ - 1; n >= 0; --n) // -1 because 0 bound.
    {
        point_t &pt = *points_iterator;

        // q1 calculation
        // begin get insertion point
        if (pt.x > q1_root_pt.x && pt.y > q1_root_pt.y) // Is point is in Q1
        {
            index_low = 0;
            index_high = q1_hull_count_;

            while (index_low < index_high - 1)
            {
                index = ((index_high - index_low) >> 1) + index_low;

                if (pt.x <= q1p_hull_points_[index].x && pt.y <= q1p_hull_points_[index].y)
                {
                    goto current_point_not_part_of_q1_hull; // No calc needed
                }

                if (pt.x > q1p_hull_points_[index].x)
                {
                    index_high = index;
                    continue;
                }

                if (pt.x < q1p_hull_points_[index].x)
                {
                    index_low = index;
                    continue;
                }

                index_low = index - 1;
                index_high = index + 1;
                break;
            }

            // Here index_low should contains the index where the point should be inserted
            // if calculation does not invalidate it.

            if (!rightTurn(q1p_hull_points_[index_low], q1p_hull_points_[index_high], pt))
            {
                goto current_point_not_part_of_q1_hull;
            }

            // HERE: We should insert a new candidate as a Hull Point (until a new one could invalidate this one, if
            // any).

            // index_low is the index of the point before the place where the new point should be inserted as the new
            // candidate of ConveHull Point. index_high is the index of the point after the place where the new point
            // should be inserted as the new candidate of ConveHull Point. But index_low and index_high can change
            // because it could invalidate many points before or after.

            // Find lower bound (remove point invalidate by the new one that come before)
            while (index_low > 0)
            {
                if (rightTurn(q1p_hull_points_[index_low - 1], pt, q1p_hull_points_[index_low]))
                {
                    break; // We found the lower index limit of points to keep. The new point should be added right
                           // after index_low.
                }
                --index_low;
            }

            // Find upper bound (remove point invalidate by the new one that come after)
            int max_index_high = q1_hull_count_ - 1;
            while (index_high < max_index_high)
            {
                if (rightTurn(pt, q1p_hull_points_[index_high + 1], q1p_hull_points_[index_high]))
                {
                    break; // We found the higher index limit of points to keep. The new point should be added right
                           // before index_high.
                }
                ++index_high;
            }

            if (index_low + 1 == index_high)
            {
                insertPoint(q1p_hull_points_, index_low + 1, pt, q1_hull_count_);
                goto next_point;
            }
            else if (index_low + 2 == index_high) // Don't need to insert, just replace at index + 1
            {
                q1p_hull_points_[index_low + 1] = *points_iterator;
                goto next_point;
            }
            else
            {
                q1p_hull_points_[index_low + 1] = *points_iterator;
                removeRange(q1p_hull_points_, index_low + 2, index_high - 1, q1_hull_count_);
                goto next_point;
            }
        }

    current_point_not_part_of_q1_hull:

        // q2 calculation
        // Begin get insertion point
        if (pt.x < q2_root_pt.x && pt.y > q2_root_pt.y) // Is point is in q2
        {
            index_low = 0;
            index_high = q2_hull_count_;

            while (index_low < index_high - 1)
            {
                index = ((index_high - index_low) >> 1) + index_low;

                if (pt.x >= q2p_hull_points_[index].x && pt.y <= q2p_hull_points_[index].y)
                {
                    goto current_point_not_part_of_q2_hull; // No calc needed
                }

                if (pt.x > q2p_hull_points_[index].x)
                {
                    index_high = index;
                    continue;
                }

                if (pt.x < q2p_hull_points_[index].x)
                {
                    index_low = index;
                    continue;
                }

                index_low = index - 1;
                index_high = index + 1;
                break;
            }

            // Here index_low should contains the index where the point should be inserted
            // if calculation does not invalidate it.

            if (!rightTurn(q2p_hull_points_[index_low], q2p_hull_points_[index_high], pt))
            {
                goto current_point_not_part_of_q2_hull;
            }

            // HERE: We should insert a new candidate as a Hull Point (until a new one could invalidate this one, if
            // any).

            // index_low is the index of the point before the place where the new point should be inserted as the new
            // candidate of ConveHull Point. index_high is the index of the point after the place where the new point
            // should be inserted as the new candidate of ConveHull Point. But index_low and index_high can change
            // because it could invalidate many points before or after.

            // Find lower bound (remove point invalidate by the new one that come before)
            while (index_low > 0)
            {
                if (rightTurn(q2p_hull_points_[index_low - 1], pt, q2p_hull_points_[index_low]))
                {
                    break; // We found the lower index limit of points to keep. The new point should be added right
                           // after index_low.
                }
                --index_low;
            }

            // Find upper bound (remove point invalidate by the new one that come after)
            int max_index_high = q2_hull_count_ - 1;
            while (index_high < max_index_high)
            {
                if (rightTurn(pt, q2p_hull_points_[index_high + 1], q2p_hull_points_[index_high]))
                {
                    break; // We found the higher index limit of points to keep. The new point should be added right
                           // before index_high.
                }
                ++index_high;
            }

            if (index_low + 1 == index_high)
            {
                insertPoint(q2p_hull_points_, index_low + 1, pt, q2_hull_count_);
                goto next_point;
            }
            else if (index_low + 2 == index_high) // Don't need to insert, just replace at index + 1
            {
                q2p_hull_points_[index_low + 1] = *points_iterator;
                goto next_point;
            }
            else
            {
                q2p_hull_points_[index_low + 1] = *points_iterator;
                removeRange(q2p_hull_points_, index_low + 2, index_high - 1, q2_hull_count_);
                goto next_point;
            }
        }

    current_point_not_part_of_q2_hull:

        // q3 calculation
        // begin get insertion point
        if (pt.x < q3_root_pt.x && pt.y < q3_root_pt.y) // Is point is in q3
        {
            index_low = 0;
            index_high = q3_hull_count_;

            while (index_low < index_high - 1)
            {
                index = ((index_high - index_low) >> 1) + index_low;

                if (pt.x >= q3p_hull_points_[index].x && pt.y >= q3p_hull_points_[index].y)
                {
                    goto current_point_not_part_of_q3_hull; // No calc needed
                }

                if (pt.x < q3p_hull_points_[index].x)
                {
                    index_high = index;
                    continue;
                }

                if (pt.x > q3p_hull_points_[index].x)
                {
                    index_low = index;
                    continue;
                }

                index_low = index - 1;
                index_high = index + 1;
                break;
            }

            // Here index_low should contains the index where the point should be inserted
            // if calculation does not invalidate it.

            if (!rightTurn(q3p_hull_points_[index_low], q3p_hull_points_[index_high], pt))
            {
                goto current_point_not_part_of_q3_hull;
            }

            // HERE: We should insert a new candidate as a Hull Point (until a new one could invalidate this one, if
            // any).

            // index_low is the index of the point before the place where the new point should be inserted as the new
            // candidate of ConveHull Point. index_high is the index of the point after the place where the new point
            // should be inserted as the new candidate of ConveHull Point. But index_low and index_high can change
            // because it could invalidate many points before or after.

            // Find lower bound (remove point invalidate by the new one that come before)
            while (index_low > 0)
            {
                if (rightTurn(q3p_hull_points_[index_low - 1], pt, q3p_hull_points_[index_low]))
                {
                    break; // We found the lower index limit of points to keep. The new point should be added right
                           // after index_low.
                }
                --index_low;
            }

            // Find upper bound (remove point invalidate by the new one that come after)
            int max_index_high = q3_hull_count_ - 1;
            while (index_high < max_index_high)
            {
                if (rightTurn(pt, q3p_hull_points_[index_high + 1], q3p_hull_points_[index_high]))
                {
                    break; // We found the higher index limit of points to keep. The new point should be added right
                           // before index_high.
                }
                ++index_high;
            }

            if (index_low + 1 == index_high)
            {
                insertPoint(q3p_hull_points_, index_low + 1, pt, q3_hull_count_);
                goto next_point;
            }
            else if (index_low + 2 == index_high) // Don't need to insert, just replace at index + 1
            {
                q3p_hull_points_[index_low + 1] = *points_iterator;
                goto next_point;
            }
            else
            {
                q3p_hull_points_[index_low + 1] = *points_iterator;
                removeRange(q3p_hull_points_, index_low + 2, index_high - 1, q3_hull_count_);
                goto next_point;
            }
        }

    current_point_not_part_of_q3_hull:
        // q4 calculation
        // begin get insertion point
        if (pt.x > q4_root_pt.x && pt.y < q4_root_pt.y) // Is point is in q4
        {
            index_low = 0;
            index_high = q4_hull_count_;

            while (index_low < index_high - 1)
            {
                index = ((index_high - index_low) >> 1) + index_low;

                if (pt.x <= q4p_hull_points_[index].x && pt.y >= q4p_hull_points_[index].y)
                {
                    goto current_point_not_part_of_q4_hull; // No calc needed
                }

                if (pt.x < q4p_hull_points_[index].x)
                {
                    index_high = index;
                    continue;
                }

                if (pt.x > q4p_hull_points_[index].x)
                {
                    index_low = index;
                    continue;
                }

                index_low = index - 1;
                index_high = index + 1;
                break;
            }

            // Here index_low should contains the index where the point should be inserted
            // if calculation does not invalidate it.

            if (!rightTurn(q4p_hull_points_[index_low], q4p_hull_points_[index_high], pt))
            {
                goto current_point_not_part_of_q4_hull;
            }

            // HERE: We should insert a new candidate as a Hull Point (until a new one could invalidate this one, if
            // any).

            // index_low is the index of the point before the place where the new point should be inserted as the new
            // candidate of ConveHull Point. index_high is the index of the point after the place where the new point
            // should be inserted as the new candidate of ConveHull Point. But index_low and index_high can change
            // because it could invalidate many points before or after.

            // Find lower bound (remove point invalidate by the new one that come before)
            while (index_low > 0)
            {
                if (rightTurn(q4p_hull_points_[index_low - 1], pt, q4p_hull_points_[index_low]))
                {
                    break; // We found the lower index limit of points to keep. The new point should be added right
                           // after index_low.
                }
                --index_low;
            }

            // Find upper bound (remove point invalidate by the new one that come after)
            int max_index_high = q4_hull_count_ - 1;
            while (index_high < max_index_high)
            {
                if (rightTurn(pt, q4p_hull_points_[index_high + 1], q4p_hull_points_[index_high]))
                {
                    break; // We found the higher index limit of points to keep. The new point should be added right
                           // before index_high.
                }
                ++index_high;
            }

            if (index_low + 1 == index_high)
            {
                insertPoint(q4p_hull_points_, index_low + 1, pt, q4_hull_count_);
                goto next_point;
            }
            else if (index_low + 2 == index_high) // Don't need to insert, just replace at index + 1
            {
                q4p_hull_points_[index_low + 1] = *points_iterator;
                goto next_point;
            }
            else
            {
                q4p_hull_points_[index_low + 1] = *points_iterator;
                removeRange(q4p_hull_points_, index_low + 2, index_high - 1, q4_hull_count_);
                goto next_point;
            }
        }

    current_point_not_part_of_q4_hull:

        // all quadrant are done
    next_point:
        ++points_iterator;
    }
}

// insert point into convex hull
void ConvexHull::insertPoint(std::deque<point_t> &p_point, int index, point_t &pt, int &count)
{
    // insert point at index
    p_point.insert(p_point.begin() + index, pt);
    ++count;
}

// remove every item from index_start to index_end inclusive
void ConvexHull::removeRange(std::deque<point_t> &p_point, int index_start, int index_end, int &count)
{
    p_point.erase(p_point.begin() + index_start, p_point.begin() + index_end + 1);
    count -= (index_end - index_start + 1);
}

void ConvexHull::getConvexHull(std::vector<point_t> &hull_points)
{
    hull_points.clear();
    if (this->count_of_point_ == 0)
    {
        return;
    }

    unsigned int index_q1_start;
    unsigned int index_q2_start;
    int index_q3_start;
    int index_q4_start;
    int index_q1_end;
    int index_q2_end;
    int index_q3_end;
    int index_q4_end;

    index_q1_start = 0;
    index_q1_end = q1_hull_count_ - 1;
    point_t point_last = q1p_hull_points_[index_q1_end];

    if (q2_hull_count_ == 1)
    {
        if (comparePoints(q2p_hull_points_[0], point_last)) //
        {
            index_q2_start = 1;
            index_q2_end = 0;
        }
        else
        {
            index_q2_start = 0;
            index_q2_end = 0;
            point_last = q2p_hull_points_[0];
        }
    }
    else
    {
        if (comparePoints(q2p_hull_points_[0], point_last))
        {
            index_q2_start = 1;
        }
        else
        {
            index_q2_start = 0;
        }
        index_q2_end = q2_hull_count_ - 1;
        point_last = q2p_hull_points_[index_q2_end];
    }

    if (q3_hull_count_ == 1)
    {
        if (comparePoints(q3p_hull_points_[0], point_last))
        {
            index_q3_start = 1;
            index_q3_end = 0;
        }
        else
        {
            index_q3_start = 0;
            index_q3_end = 0;
            point_last = q3p_hull_points_[0];
        }
    }
    else
    {
        if (comparePoints(q3p_hull_points_[0], point_last))
        {
            index_q3_start = 1;
        }
        else
        {
            index_q3_start = 0;
        }
        index_q3_end = q3_hull_count_ - 1;
        point_last = q3p_hull_points_[index_q3_end];
    }

    if (q4_hull_count_ == 1)
    {
        if (comparePoints(q4p_hull_points_[0], point_last))
        {
            index_q4_start = 1;
            index_q4_end = 0;
        }
        else
        {
            index_q4_start = 0;
            index_q4_end = 0;
            point_last = q4p_hull_points_[0];
        }
    }
    else
    {
        if (comparePoints(q4p_hull_points_[0], point_last))
        {
            index_q4_start = 1;
        }
        else
        {
            index_q4_start = 0;
        }

        index_q4_end = q4_hull_count_ - 1;
        point_last = q4p_hull_points_[index_q4_end];
    }

    if (comparePoints(q1p_hull_points_[index_q1_start], point_last))
    {
        index_q1_start++;
    }

    int count_of_final_hull_point = (index_q1_end - index_q1_start) + (index_q2_end - index_q2_start) +
                                    (index_q3_end - index_q3_start) + (index_q4_end - index_q4_start) + 4;

    if (count_of_final_hull_point <=
        1) // Case where there is only one point or many of only the same point. Auto closed if required.
    {
        hull_points = {point_last};
        return;
    }

    if (count_of_final_hull_point > 1 && should_close_the_graph_)
    {
        count_of_final_hull_point++;
    }

    hull_points.resize(count_of_final_hull_point);

    int resIndex = 0;

    for (int n = index_q1_start; n <= index_q1_end; n++)
    {
        hull_points[resIndex] = q1p_hull_points_[n];
        resIndex++;
    }

    for (int n = index_q2_start; n <= index_q2_end; n++)
    {
        hull_points[resIndex] = q2p_hull_points_[n];
        resIndex++;
    }

    for (int n = index_q3_start; n <= index_q3_end; n++)
    {
        hull_points[resIndex] = q3p_hull_points_[n];
        resIndex++;
    }

    for (int n = index_q4_start; n <= index_q4_end; n++)
    {
        hull_points[resIndex] = q4p_hull_points_[n];
        resIndex++;
    }

    if (count_of_final_hull_point > 1 && should_close_the_graph_)
    {
        hull_points[resIndex] = hull_points[0];
    }
}

} // namespace lidar_processing