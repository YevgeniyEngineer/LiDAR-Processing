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

void Segmenter::extract_initial_seeds(const containers::Vector<Point> &cloud_segment)
{
    ground_indices_.clear();

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

    ground_indices_.resize(z_max_cutoff_index);
    for (std::size_t i = 0UL; i < z_max_cutoff_index; ++i)
    {
        ground_indices_[i] = sorted_indices_[i];
    }
}

void Segmenter::fit_ground_plane(const containers::Vector<Point> &cloud_segment)
{
    ground_indices_.clear();
    obstacle_indices_.clear();

    const std::uint32_t number_of_points = cloud_segment.size();
    if (number_of_points == 0)
    {
        return;
    }

    cloud_buffer_.resize(number_of_points * 3);
    auto cloud_buffer_iterator = cloud_buffer_.begin();
    for (const auto &point : cloud_segment)
    {
        *(cloud_buffer_iterator++) = point.x;
        *(cloud_buffer_iterator++) = point.y;
        *(cloud_buffer_iterator++) = point.z;
    }

    const Eigen::Map<Eigen::MatrixXf> points_xyz{cloud_buffer_.data(), number_of_points, 3};

    extract_initial_seeds(cloud_segment);

    Plane plane_coefficients;

    for (std::uint32_t iteration = 0U; iteration < configuration_.number_of_iterations; ++iteration)
    {
        const std::size_t number_of_ground_points = ground_indices_.size();

        if (number_of_ground_points < 3)
        {
            // Failed to find ground points - treat everything as obstacles
            ground_indices_.clear();
            obstacle_indices_.resize(number_of_points);
            std::iota(obstacle_indices_.begin(), obstacle_indices_.end(), 0U);
            return;
        }

        ground_buffer_.resize(number_of_ground_points * 3);
        auto ground_buffer_iterator = ground_buffer_.begin();
        for (std::size_t i = 0UL; i < number_of_ground_points; ++i)
        {
            const auto &point = cloud_segment[ground_indices_[i]];
            *(ground_buffer_iterator++) = point.x;
            *(ground_buffer_iterator++) = point.y;
            *(ground_buffer_iterator++) = point.z;
        }
        const Eigen::Map<Eigen::MatrixXf> ground_points_xyz(ground_buffer_.data(), number_of_ground_points, 3);

        const bool successful = estimate_plane_coefficients(ground_points_xyz, plane_coefficients);

        if (!successful)
        {
            // Failed to find plane coefficients - treat everything as obstacles
            ground_indices_.clear();
            obstacle_indices_.resize(number_of_points);
            std::iota(obstacle_indices_.begin(), obstacle_indices_.end(), 0U);
            return;
        }

        const Eigen::Vector3f normal(plane_coefficients.a, plane_coefficients.b, plane_coefficients.c);

        distance_buffer_.resize(number_of_points);
        Eigen::Map<Eigen::Vector3f> distances(distance_buffer_.data(), number_of_points, 1);

        distances.noalias() = points_xyz - normal;
        distances.array() -= normal.z();

        const float scaled_orthogonal_distance_threshold = configuration_.orthogonal_distance_threshold * normal.norm();
        ground_indices_.clear();
        obstacle_indices_.clear();

        for (std::uint32_t point_index = 0U; point_index < number_of_points; ++point_index)
        {
            if (distances(point_index) < scaled_orthogonal_distance_threshold)
            {
                ground_indices_.push_back(point_index);
            }
            else
            {
                obstacle_indices_.push_back(point_index);
            }
        }
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
