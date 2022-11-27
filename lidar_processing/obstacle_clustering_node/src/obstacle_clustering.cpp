#include "obstacle_clustering.hpp"
// #include "point_labels.hpp"

namespace lidar_processing
{
// Perform clustering on the input point cloud, and place clustered clouds into pcl::PointXYZRGBL
void ObstacleClustering::clusterObstacles(const pcl::PointCloud<pcl::PointXYZRGBI> &cloud,
                                          pcl::PointCloud<pcl::PointXYZRGBL> &clustered_cloud)
{
    size_t number_of_points = cloud.size();

    // copy cloud into this cloud, that will hold intensities and labels
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<float> intensities;
    std::vector<std::uint32_t> labels(number_of_points); // initializes all elements to 0
    input_cloud->points.reserve(number_of_points);

    for (const pcl::PointXYZRGBI &point : cloud.points)
    {
        pcl::PointXYZ point_cache;
        point_cache.x = point.x;
        point_cache.y = point.y;
        point_cache.z = point.z;

        intensities.emplace_back(point.intensity);
        input_cloud->points.emplace_back(std::move(point_cache));
    }
    input_cloud->height = 1;
    input_cloud->width = input_cloud->points.size();

    // start of the clustering algorithm
    std::uint32_t segment_label = 1U;

    // construct kd-tree
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
    kdtree->setInputCloud(input_cloud);

    // iterate over each point
    for (size_t i = 0; i < number_of_points; ++i)
    {
        // check if the label is a default label
        if (labels[i] == 0U)
        {
            // perform kd-tree search
            const pcl::PointXYZ &core_point = input_cloud->points[i];
            pcl::Indices point_idx_radius_search;
            std::vector<float> point_radius_squared_distance;
            int number_of_neighbours =
                kdtree->radiusSearch(core_point, neighbour_radius_threshold_, point_idx_radius_search,
                                     point_radius_squared_distance, maximum_neighbour_points_);

            if (number_of_neighbours > 0)
            {
                // check if a non-zero label exists among nearest neighbors
                bool non_zero_label_exists = false;
                for (const auto &point_idx : point_idx_radius_search)
                {
                    if (labels[point_idx] != 0U)
                    {
                        non_zero_label_exists = true;
                        break;
                    }
                }

                // set the minimum segment label
                std::uint32_t minimum_segment_label;
                if (non_zero_label_exists)
                {
                    for (const auto &point_idx : point_idx_radius_search)
                    {
                        const auto &label = labels[point_idx];
                        if (label != 0U)
                        {
                            minimum_segment_label = std::min(label, segment_label);
                        }
                    }
                }
                else
                {
                    minimum_segment_label = segment_label;
                }

                // segment merge loop
                for (const auto &neighbour_idx : point_idx_radius_search)
                {
                    std::uint32_t label_j = labels[neighbour_idx];
                    if (label_j > minimum_segment_label)
                    {
                        for (int k = 0; k < labels.size(); ++k)
                        {
                            if (labels[k] == label_j)
                            {
                                labels[k] = minimum_segment_label;
                                std::cout << labels[k];
                            }
                        }
                    }
                }
            }
            ++segment_label;
        }
    }
}
} // namespace lidar_processing