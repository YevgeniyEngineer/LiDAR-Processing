#include "ground_segmentation.hpp"

namespace lidar_processing
{
GroundSegmenter::GroundSegmenter(std::uint32_t number_of_iterations, std::uint32_t number_of_planar_partitions,
                                 std::uint32_t number_of_lowest_point_representative_estimators, float sensor_height,
                                 float distance_threshold, float initial_seed_threshold)
    : number_of_iterations_(number_of_iterations), number_of_planar_partitions_(number_of_planar_partitions),
      number_of_lowest_point_representative_estimators_(number_of_lowest_point_representative_estimators),
      sensor_height_(sensor_height), distance_threshold_(distance_threshold),
      initial_seed_threshold_(initial_seed_threshold)
{
    if (number_of_iterations <= 0)
    {
        throw std::runtime_error("Number of iterations must be greater than 0");
    }
    if (number_of_planar_partitions <= 0)
    {
        throw std::runtime_error("Number of planar partitions must be greater than 0");
    }
    if (number_of_lowest_point_representative_estimators < 3)
    {
        throw std::runtime_error("Number of lowest point representative estimators must contain at least 3 points");
    }
    if (distance_threshold < 0)
    {
        throw std::runtime_error("Distance threshold must be non-negative");
    }
    if (initial_seed_threshold < 0)
    {
        throw std::runtime_error("Initial seed threshold must be non-negative");
    }
}

void GroundSegmenter::formPlanarPartitions(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                           std::vector<pcl::PointCloud<pcl::PointXYZ>> &cloud_segments)
{
    if (cloud.empty())
    {
        return;
    }

    const auto &number_of_points = cloud.size();

    // sort and get indices corresponding to the point cloud that is sorted in increasing x-order
    std::vector<size_t> sorted_indices(number_of_points);

    // fill vector with increasing values, starting from 0
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    // sort indices using stable sort
    const auto &cloud_points = cloud.points;
    std::sort(std::execution::par, sorted_indices.begin(), sorted_indices.end(),
              [&cloud_points](const std::size_t &idx_1, const std::size_t &idx_2) -> bool {
                  return cloud_points[idx_1].x < cloud_points[idx_2].x;
              });

    // iterate over sorted indices and partition point cloud
    std::size_t elements_within_segment = number_of_points / number_of_planar_partitions_;
    std::size_t idx_low = 0;
    std::size_t idx_high = elements_within_segment;
    cloud_segments.reserve(number_of_planar_partitions_);

    for (std::size_t segment_no = 0; segment_no < number_of_planar_partitions_; ++segment_no)
    {
        auto cloud_segment = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_segment->points.reserve(idx_high - idx_low + 1);

        for (std::size_t sorted_idx_no = idx_low; sorted_idx_no < idx_high; ++sorted_idx_no)
        {
            const auto &cloud_point = cloud_points[sorted_indices[sorted_idx_no]];
            cloud_segment->points.push_back(pcl::PointXYZ{cloud_point.x, cloud_point.y, cloud_point.z});
        }

        cloud_segments.push_back(std::move(*cloud_segment));
        cloud_segment.reset();

        // update indices
        idx_low = idx_high;
        idx_high = std::min(idx_low + elements_within_segment, number_of_points);
    }
}

GroundPlane estimatePlane(const Eigen::MatrixXf &points_xyz)
{
    // Eigendecomposition of the covariance matrix - returns ordered eigenvalues in the increasing order
    // Second column of V matrix from SVD decomposition corresponds to normal vector to fitted plane
    // See: https://eigen.tuxfamily.org/dox/group__DenseDecompositionBenchmark.html
    // See: https://eigen.tuxfamily.org/dox-devel/group__TopicLinearAlgebraDecompositions.html#note3

    const auto number_of_points = points_xyz.rows();
    if (number_of_points < 3)
    {
        throw std::runtime_error("Cannot estimate plane parameters for less than three points");
    }

    // Calculate centroid
    Eigen::Vector3f centroid = points_xyz.colwise().mean();

    // Shift the origin of the point cloud towards the centroid
    Eigen::MatrixXf centered_points = points_xyz.rowwise() - centroid.transpose();

    // Compute the covariance matrix
    Eigen::Matrix3f covariance_matrix = centered_points.transpose() * centered_points;

    // Compute the SVD of the covariance matrix
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance_matrix, Eigen::ComputeThinV);

    // The normal of the plane is the unit singular vector corresponding to the smallest singular value,
    // which is the last column in the V matrix.
    Eigen::Vector3f normal = svd.matrixV().col(2);

    // Get plane parameters
    float a = normal(0);
    float b = normal(1);
    float c = normal(2);
    float d = -normal.dot(centroid);

    return GroundPlane(a, b, c, d);
}

void GroundSegmenter::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices)
{
    if (cloud.empty())
    {
        return;
    }

    const auto &number_of_points = cloud.points.size();

    // copy points in cloud copy
    std::vector<pcl::PointXYZ> cloud_copy(cloud.points.cbegin(), cloud.points.cend());

    // get indices of a sorted array
    std::vector<int> cloud_indices(number_of_points);
    std::iota(cloud_indices.begin(), cloud_indices.end(), 0);

    // apply sorting on copied point cloud
    std::sort(std::execution::par, cloud_indices.begin(), cloud_indices.end(),
              [&](const int &index_1, const int &index_2) -> bool {
                  return (cloud_copy[index_1].z < cloud_copy[index_2].z);
              });

    // remove outlier points located below ground
    float negative_offset_threshold = -1.5 * sensor_height_;
    int lower_cutoff_index = 0;
    for (int i = 0; i < cloud_indices.size(); ++i)
    {
        if (cloud.points[cloud_indices[i]].z > negative_offset_threshold)
        {
            lower_cutoff_index = i;
            break;
        }
    }
    cloud_indices.erase(cloud_indices.begin(), cloud_indices.begin() + lower_cutoff_index);
    if (cloud_indices.empty())
    {
        return;
    }

    // find the average height of the lowest point representatives
    float lowest_point_representative_height = 0.0F;
    int number_of_estimators = std::min(static_cast<int>(cloud_indices.size()),
                                        static_cast<int>(number_of_lowest_point_representative_estimators_));
    for (int i = 0; i < number_of_estimators; ++i)
    {
        lowest_point_representative_height += cloud.points[cloud_indices[i]].z;
    }
    lowest_point_representative_height /= number_of_estimators;

    // filter points that have height less that lowest_point_representative_height + initial_seed_threshold_
    float cutoff_height = lowest_point_representative_height + initial_seed_threshold_;
    int upper_cutoff_index = 0;
    for (int i = 0; i < cloud_indices.size(); ++i)
    {
        if (cloud.points[cloud_indices[i]].z > cutoff_height)
        {
            upper_cutoff_index = i;
            break;
        }
    }

    // move indices
    indices.clear();
    indices.insert(indices.end(), std::make_move_iterator(cloud_indices.begin()),
                   std::make_move_iterator(cloud_indices.begin() + upper_cutoff_index));

#if DEBUG
    std::cout << "Number of seed points: " << indices.size() << std::endl;
#endif
}

void GroundSegmenter::fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZ> &ground_cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud_segment)
{
    const std::size_t &number_of_points = cloud_segment.points.size();
    if (number_of_points == 0)
    {
        return;
    }

    // convert input_cloud to a matrix form
    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
    for (std::size_t i = 0; i < number_of_points; ++i)
    {
        const pcl::PointXYZ &point = cloud_segment[i];
        points_xyz.row(i) << point.x, point.y, point.z;
    }

    // to speed up emplacing points (set to maximum capacity)
    std::vector<int> ground_cloud_indices;
    ground_cloud_indices.reserve(number_of_points);

    std::vector<int> obstacle_cloud_indices;
    obstacle_cloud_indices.reserve(number_of_points);

    // extract initial seeds (ground points)
    extractInitialSeeds(cloud_segment, ground_cloud_indices);

    // iterate for number of iterations to refine ground plane fit
    for (int iter_no = 0; iter_no < number_of_iterations_; ++iter_no)
    {
        // convert ground points PCL PointCloud to Eigen matrix
        std::size_t number_of_ground_points = ground_cloud_indices.size();
        if (number_of_ground_points == 0)
        {
            break;
        }

        Eigen::MatrixXf ground_points_xyz(number_of_ground_points, 3); // N x 3
        for (int i = 0; i < number_of_ground_points; ++i)
        {
            ground_points_xyz.row(i) = points_xyz.row(ground_cloud_indices[i]);
        }

        // estimate plane parameters a, b, c, d
        const GroundPlane &plane = estimatePlane(ground_points_xyz);

        // calculate distance from each point to the plane
        Eigen::Vector3f normal(3);
        normal << plane.a, plane.b, plane.c; // 3 x 1

        // ax0 + by0 + cz0 - d
        Eigen::VectorXf distances_unnormalized = (points_xyz * normal).array() - plane.d;

        // D * sqrt(a^2 + b^2 + c^2)
        const float &scaled_distance_threshold =
            distance_threshold_ * std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);

        // set indices for ground and obstacle points
        ground_cloud_indices.clear();
        obstacle_cloud_indices.clear();
        for (int k = 0; k < number_of_points; ++k)
        {
            if (distances_unnormalized[k] < scaled_distance_threshold)
            {
                ground_cloud_indices.push_back(k);
            }
            else
            {
                obstacle_cloud_indices.push_back(k);
            }
        }
    }

    if (ground_cloud_indices.size() != 0)
    {
        // copy points based on indices (ground cloud)
        ground_cloud_segment.clear();
        ground_cloud_segment.reserve(ground_cloud_indices.size());
        for (const auto &ground_cloud_index : ground_cloud_indices)
        {
            ground_cloud_segment.points.emplace_back(cloud_segment.points[ground_cloud_index]);
        }
        ground_cloud_segment.width = ground_cloud_segment.points.size();
        ground_cloud_segment.height = 1;

        // copy points based on indices (obstacle cloud)
        obstacle_cloud_segment.clear();
        obstacle_cloud_segment.reserve(obstacle_cloud_indices.size());
        for (const auto &obstacle_cloud_index : obstacle_cloud_indices)
        {
            obstacle_cloud_segment.points.emplace_back(cloud_segment.points[obstacle_cloud_index]);
        }
        obstacle_cloud_segment.width = obstacle_cloud_segment.points.size();
        obstacle_cloud_segment.height = 1;
    }
    else // no ground points - everything is an obstacle
    {
        obstacle_cloud_segment.clear();
        obstacle_cloud_segment.reserve(number_of_points);
        for (const auto &point : cloud_segment.points)
        {
            obstacle_cloud_segment.points.emplace_back(point);
        }
        obstacle_cloud_segment.width = obstacle_cloud_segment.points.size();
        obstacle_cloud_segment.height = 1;
    }
}

void GroundSegmenter::combineSegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &cloud_segments,
                                             const SegmentationLabels &segmentation_label,
                                             pcl::PointCloud<pcl::PointXYZRGBL> &segmented_cloud) const
{
    ColorRGB color;
    if (segmentation_label == SegmentationLabels::GROUND)
    {
        color = ColorRGB(220, 220, 220);
    }
    else
    {
        color = ColorRGB(0, 255, 0);
    }

    // clear old points
    segmented_cloud.clear();

    // preallocate memory
    std::uint32_t number_of_elements = 0;
    for (const auto &cloud_segment : cloud_segments)
    {
        number_of_elements += cloud_segment.size();
    }
    segmented_cloud.reserve(number_of_elements);

    // copy elements from each segment into PointXYZRGBL
    pcl::PointXYZRGBL point_cache;
    point_cache.r = color.r;
    point_cache.g = color.g;
    point_cache.b = color.b;
    point_cache.a = 255;
    point_cache.label = static_cast<decltype(point_cache.label)>(segmentation_label);

    for (const auto &cloud_segment : cloud_segments)
    {
        for (const auto &point : cloud_segment.points)
        {
            point_cache.x = point.x;
            point_cache.y = point.y;
            point_cache.z = point.z;
            segmented_cloud.points.push_back(point_cache);
        }
    }
    segmented_cloud.height = 1;
    segmented_cloud.width = segmented_cloud.points.size();
}

} // namespace lidar_processing