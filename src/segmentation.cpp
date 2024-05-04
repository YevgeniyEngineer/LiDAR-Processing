#include "segmentation.hpp"

// STL
#include <algorithm>
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

bool Segmenter::estimate_plane_coefficients(const Eigen::Map<Eigen::MatrixXf> &ground_points_xyz,
                                            Plane &plane_coefficients)
{
    // https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
    // https://eigen.tuxfamily.org/dox-devel/group__TopicLinearAlgebraDecompositions.html#note3
    // https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

    const auto number_of_points = ground_points_xyz.rows();

    if (number_of_points < 3)
    {
        return false;
    }

    const Eigen::RowVector3f centroid = ground_points_xyz.colwise().mean();

    centered_points_buffer_.resize(static_cast<std::size_t>(number_of_points) * 3UL);
    Eigen::Map<Eigen::MatrixXf> centered_points(centered_points_buffer_.data(), number_of_points, 3);

    centered_points.noalias() = ground_points_xyz.rowwise() - centroid;

    Eigen::Matrix3f covariance = centered_points.transpose() * centered_points;
    covariance /= static_cast<float>(number_of_points - 1);

    svd_solver_.compute(covariance);

    const Eigen::Vector3f normal = svd_solver_.matrixV().col(2);

    plane_coefficients.a = normal(0);
    plane_coefficients.b = normal(1);
    plane_coefficients.c = normal(2);
    plane_coefficients.d = normal.dot(centroid);

    return true;
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
    std::size_t index_low = 0UL;
    std::size_t index_high = number_of_points_within_segment;

    cloud_segments.reserve(configuration_.number_of_planar_partitions);
    cloud_segments.clear();

    for (std::size_t segment_index = 0UL; segment_index < configuration_.number_of_planar_partitions; ++segment_index)
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

void Segmenter::extract_initial_seeds(const containers::Vector<Point> &cloud_segment,
                                      containers::Vector<std::uint32_t> &ground_indices)
{
    ground_indices.clear();

    if (cloud_segment.empty())
    {
        return;
    }

    const std::size_t number_of_points = cloud_segment.size();

    sorted_indices_.resize(number_of_points);
    std::iota(sorted_indices_.begin(), sorted_indices_.end(), 0U);

    std::sort(sorted_indices_.begin(), sorted_indices_.end(),
              [&cloud_segment](const auto &index_1, const auto &index_2) -> bool {
                  return cloud_segment[index_1].z < cloud_segment[index_2].z;
              });

    // -1.5 to account for points below ground level
    const float z_min_lowest_point_representative = -1.5F * configuration_.sensor_height_m;
    std::size_t z_min_cutoff_index = 0UL;
    for (std::size_t i = 0UL; i < sorted_indices_.size(); ++i)
    {
        if (cloud_segment[sorted_indices_[i]].z > z_min_lowest_point_representative)
        {
            z_min_cutoff_index = i;
            break;
        }
    }

    sorted_indices_.erase(sorted_indices_.begin(), sorted_indices_.begin() + z_min_cutoff_index);

    if (sorted_indices_.empty())
    {
        return;
    }

    float z_mean_lowest_point_representative = 0.0F;
    const std::size_t number_of_lowest_point_representatives = std::min(
        sorted_indices_.size(), static_cast<std::size_t>(configuration_.number_of_lower_point_representatives));

    for (std::size_t i = 0UL; i < number_of_lowest_point_representatives; ++i)
    {
        z_mean_lowest_point_representative += cloud_segment[sorted_indices_[i]].z;
    }
    z_mean_lowest_point_representative /= number_of_lowest_point_representatives;

    const float z_max_lowest_point_representative =
        z_mean_lowest_point_representative + configuration_.initial_seed_threshold;

    std::size_t z_max_cutoff_index = 0UL;
    for (std::size_t i = 0UL; i < sorted_indices_.size(); ++i)
    {
        if (cloud_segment[sorted_indices_[i]].z > z_max_lowest_point_representative)
        {
            z_max_cutoff_index = i;
            break;
        }
    }

    ground_indices.resize(z_max_cutoff_index);
    for (std::size_t i = 0UL; i < z_max_cutoff_index; ++i)
    {
        ground_indices[i] = sorted_indices_[i];
    }
}

void Segmenter::fit_ground_plane(const containers::Vector<Point> &cloud_segment,
                                 containers::Vector<std::uint32_t> &ground_indices,
                                 containers::Vector<std::uint32_t> &obstacle_indices)
{
    ground_indices.clear();
    obstacle_indices.clear();

    if (cloud_segment.empty())
    {
        return;
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
