#ifndef LIDAR_PROCESSING__SEGMENTATION_HPP
#define LIDAR_PROCESSING__SEGMENTATION_HPP

// Containers
#include "vector.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <cstdint>

namespace lidar_processing
{
enum class SegmentationLabel : std::uint32_t
{
    UNKNOWN = 0U,
    GROUND,
    OBSTACLE
};

struct SegmentationConfiguration final
{
    float sensor_height_m{1.73F};
    float orthogonal_distance_threshold{0.3F};
    float initial_seed_threshold{0.6F};
    std::uint32_t number_of_iterations{3U};
    std::uint32_t number_of_planar_partitions{2U};
    std::uint32_t number_of_lower_point_representatives{5000U};
};

class Segmenter final
{
  public:
    Segmenter() = default;
    ~Segmenter() = default;

    void update_configuration(const SegmentationConfiguration &configuration);

    void reserve_memory(std::uint32_t number_of_points);

    template <typename PointT>
    void segment(const pcl::PointCloud<PointT> &cloud_in, std::vector<SegmentationLabel> &labels,
                 pcl::PointCloud<PointT> &ground_cloud, pcl::PointCloud<PointT> &obstacle_cloud);

  private:
    struct Point final
    {
        float x{0U};
        float y{0U};
        float z{0U};
        SegmentationLabel label{SegmentationLabel::UNKNOWN};
        std::uint32_t index{0U};

        Point() = default;
        Point(float x, float y, float z, SegmentationLabel label, std::uint32_t index)
            : x(x), y(y), z(z), label(label), index(index)
        {
        }
    };

    SegmentationConfiguration configuration_;
    containers::Vector<std::uint32_t> sorted_indices_;
    containers::Vector<containers::Vector<pcl::PointXYZ>> cloud_segments_;
    containers::Vector<std::uint32_t> ground_indices_;
    containers::Vector<std::uint32_t> obstacle_indices_;

    template <typename PointT>
    void form_planar_partitions(const pcl::PointCloud<PointT> &cloud_in,
                                containers::Vector<containers::Vector<Point>> &cloud_segments);
    void extract_initial_seeds(const containers::Vector<Point> &cloud_segment,
                               containers::Vector<std::uint32_t> &ground_indices);
    void fit_ground_plane();
    void combine_segmented_points();
};

extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                        std::vector<SegmentationLabel> &labels,
                                        pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
                                        pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud);

extern template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                        std::vector<SegmentationLabel> &labels,
                                        pcl::PointCloud<pcl::PointXYZI> &ground_cloud,
                                        pcl::PointCloud<pcl::PointXYZI> &obstacle_cloud);

extern template void Segmenter::form_planar_partitions(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                                       containers::Vector<containers::Vector<Point>> &cloud_partitions);

extern template void Segmenter::form_planar_partitions(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                                       containers::Vector<containers::Vector<Point>> &cloud_partitions);

} // namespace lidar_processing

#endif // LIDAR_PROCESSING__SEGMENTATION_HPP
