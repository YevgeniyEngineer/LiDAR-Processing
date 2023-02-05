#ifndef QUAD_TREE_HPP_
#define QUAD_TREE_HPP_

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

/**** This is a basic implementation of QuadTree. It requires more work to be useable ****/

// Structure to hold 2D cartesian point
struct Point
{
    float x, y;
    Point() = default;
    Point(float _x, float _y) : x(_x), y(_y){};
    ~Point() = default;
};

// Axis-aligned bounding box
struct BoundingBox
{
    Point center;
    float half_width;
    float x_min, x_max, y_min, y_max;

    explicit BoundingBox(const Point &_center, float _half_width) : center(_center), half_width(_half_width)
    {
        this->x_min = center.x - half_width;
        this->x_max = center.x + half_width;
        this->y_min = center.y - half_width;
        this->y_max = center.y + half_width;
    };
    explicit BoundingBox(const BoundingBox &rhs)
    {
        this->center = rhs.center;
        this->half_width = rhs.half_width;
        this->x_min = rhs.x_min;
        this->x_max = rhs.x_max;
        this->y_min = rhs.y_min;
        this->y_max = rhs.y_max;
    }
    ~BoundingBox() = default;

    bool containsPoint(const Point &point) const
    {
        if (point.x < x_min || point.x > x_max)
        {
            return false;
        }
        if (point.y < y_min || point.y > y_max)
        {
            return false;
        }
        return true;
    }

    bool intersectsBoundingBox(const BoundingBox &bbox) const
    {
        if (bbox.x_min > x_max || bbox.x_max < x_min)
        {
            return false;
        }
        if (bbox.y_min > y_max || bbox.y_max < y_min)
        {
            return false;
        }
        return true;
    }
};

class QuadTree
{
    constexpr static const unsigned int NODE_CAPACITY = 4;

  private:
    // Represents boundaries of this quad tree
    BoundingBox boundary_;

    // Points in this QuadTree node
    std::vector<Point> points_;

    // Children of the QuadTree node
    std::unique_ptr<QuadTree> north_west_;
    std::unique_ptr<QuadTree> north_east_;
    std::unique_ptr<QuadTree> south_west_;
    std::unique_ptr<QuadTree> south_east_;

  public:
    explicit QuadTree(const BoundingBox &boundary)
        : boundary_(boundary), north_west_(nullptr), north_east_(nullptr), south_west_(nullptr),
          south_east_(nullptr){
              // std::cout << "Created new quad" << std::endl;
          };
    ~QuadTree()
    {
        north_west_.release();
        north_east_.release();
        south_west_.release();
        south_east_.release();
        points_.clear();
    };

    // Insert a point into the QuadTree
    bool insert(const Point &point)
    {
        // Ignore the object that does not belong to this quad tree
        if (!boundary_.containsPoint(point))
        {
            // Object cannot be added
            return false;
        }

        // If there is space in quad tree and
        // If it does not have subdivisions,
        // add the object here
        if (points_.size() < NODE_CAPACITY && north_west_.get() == nullptr)
        {
            points_.push_back(point);
            return true;
        }
        // Otherwise, subdivide and the add the point to
        // whichever node will accept it
        if (north_west_.get() == nullptr)
        {
            subdivide();
        }
        // We have to add the points/data contained in this
        // quad array to the new quads if we only want
        // the last node to hold the data
        if (north_west_->insert(point))
        {
            return true;
        }
        if (north_east_->insert(point))
        {
            return true;
        }
        if (north_west_->insert(point))
        {
            return true;
        }
        if (south_east_->insert(point))
        {
            return true;
        }
        // Otherwise, the point cannot be inserted for some
        // unknown reason (this should never happen)
        return false;
    }

    // Create four children that fully divide this quad into four quads of equal area
    void subdivide()
    {
        // Divide boundaries of current node
        float half_width = boundary_.half_width / 2.0f;

        // Find center points in each quadrant
        float x_west = boundary_.center.x - half_width;
        float x_east = boundary_.center.x + half_width;
        float y_south = boundary_.center.y - half_width;
        float y_north = boundary_.center.y + half_width;

        // Assign center points to each quadrant
        Point north_west_center{x_west, y_north};
        Point north_east_center{x_east, y_north};
        Point south_west_center{x_west, y_south};
        Point south_east_center{x_east, y_south};

        // Form bounding box specifying boundary of each quadrant
        BoundingBox north_west_boundary(north_west_center, half_width);
        BoundingBox north_east_boundary(north_east_center, half_width);
        BoundingBox south_west_boundary(south_west_center, half_width);
        BoundingBox south_east_boundary(south_east_center, half_width);

        // Create new quads
        north_west_ = std::make_unique<QuadTree>(north_west_boundary);
        north_east_ = std::make_unique<QuadTree>(north_east_boundary);
        south_west_ = std::make_unique<QuadTree>(south_west_boundary);
        south_east_ = std::make_unique<QuadTree>(south_east_boundary);
    }

    // Find all points contained within range
    void queryRange(const BoundingBox &range_boundary, std::vector<Point> &range_points)
    {
        // Terminate here, if there are no children
        if (north_west_.get() == nullptr)
        {
            return;
        }

        // Automatically abort if the range does not intersect this quad
        if (!boundary_.intersectsBoundingBox(range_boundary))
        {
            // Points are not within bounding box boundary
            return;
        }

        // Check objects at this quad level
        for (const auto &point : points_)
        {
            if (range_boundary.containsPoint(point))
            {
                range_points.push_back(point);
            }
        }

        // Otherwise, add the points from the children
        north_west_->queryRange(range_boundary, range_points);
        north_east_->queryRange(range_boundary, range_points);
        south_west_->queryRange(range_boundary, range_points);
        south_east_->queryRange(range_boundary, range_points);
    }
};

#endif // QUAD_TREE_HPP_