#include "segmentation.hpp"

// STL
#include <numeric>

namespace lidar_processing
{
void Segmenter::update_configuration(const SegmentationConfiguration &configuration)
{
    configuration_ = configuration;
}

template <typename PointT>
void Segmenter::segment(const pcl::PointCloud<PointT> &cloud_in, std::vector<SegmentationLabel> &labels,
                        pcl::PointCloud<PointT> &ground_cloud, pcl::PointCloud<PointT> &obstacle_cloud)
{
}

template <typename PointT>
void Segmenter::form_planar_partitions(const pcl::PointCloud<PointT> &cloud_in,
                                       containers::Vector<containers::Vector<Point>> &cloud_segments)
{
    cloud_segments.clear();

    if (cloud_in.empty())
    {
        return;
    }

    const std::size_t number_of_points = cloud_in.size();

    sorted_indices_.resize(number_of_points);
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0U);

    const auto &cloud_points = cloud_in.points;
    std::sort(sorted_indices_.begin(), sorted_indices_.end(),
              [&cloud_points](const auto &index_1, const auto &index_2) -> bool {
                  return cloud_points[index_1].x < cloud_points[index_2].x;
              });

    const std::size_t number_of_points_within_segment = number_of_points / configuration_.number_of_planar_partitions;
    std::size_t index_low = 0U;
    std::size_t index_high = number_of_points_within_segment;

    cloud_segments.reserve(configuration_.number_of_planar_partitions);
    cloud_segments.clear();

    for (std::size_t segment_index = 0U; segment_index < configuration_.number_of_planar_partitions; ++segment_index)
    {
        cloud_segments.resize(cloud_segments.size() + 1UL);
        auto &cloud_segment = cloud_segments.back();

        cloud_segment.reserve(index_high - index_low + 1UL);
        cloud_segment.clear();

        for (std::size_t index = index_low; index < index_high; ++index)
        {
            const auto &point_index = sorted_indices_[index];
            const auto &cloud_point = cloud_points[point_index];
            cloud_segment.emplace_back(cloud_point.x, cloud_point.y, cloud_point.z, SegmentationLabel::UNKNOWN,
                                       point_index);
        }

        index_low = index_high;
        index_high = std::min(index_low + number_of_points_within_segment, number_of_points);
    }
}

template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<SegmentationLabel> &labels,
                                 pcl::PointCloud<pcl::PointXYZ> &ground_cloud,
                                 pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud);

template void Segmenter::segment(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                 std::vector<SegmentationLabel> &labels, pcl::PointCloud<pcl::PointXYZI> &ground_cloud,
                                 pcl::PointCloud<pcl::PointXYZI> &obstacle_cloud);

template void Segmenter::form_planar_partitions(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                                containers::Vector<containers::Vector<Point>> &cloud_partitions);

template void Segmenter::form_planar_partitions(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                                                containers::Vector<containers::Vector<Point>> &cloud_partitions);

} // namespace lidar_processing
