#pragma once
#ifndef OBSTACLE_CLUSTERING_HPP_
#define OBSTACLE_CLUSTERING_HPP_

namespace lidar_processing
{
class ObstacleClustering
{
  public:
    ObstacleClustering();

    ~ObstacleClustering() = default;

    void clusterObstacles();

  private:
};
} // namespace lidar_processing

#endif // OBSTACLE_CLUSTERING_HPP_