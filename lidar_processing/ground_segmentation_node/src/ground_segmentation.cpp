#include "ground_segmentation.hpp"
#include "point_labels.hpp"
namespace lidar_processing
{
// Partition points into random segments
// Partitions along x-axis, that is the direction of forward vehicle movement
void GroundSegmentation::formSegments(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_segments)
{
    if (cloud.empty())
    {
        return;
    }

    const size_t &number_of_points = cloud.points.size();

    // sort and get indices corresponding to the point cloud that is sorted in increasing x-order
    std::vector<size_t> sorted_indices(number_of_points);

    // fill vector with increasing values, starting from 0
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    // sort indices using stable sort
    const auto &cloud_points = cloud.points;
    std::stable_sort(sorted_indices.begin(), sorted_indices.end(), [&cloud_points](size_t idx_1, size_t idx_2) -> bool {
        return cloud_points[idx_1].x < cloud_points[idx_2].x;
    });

    // iterate over sorted indices and partition point cloud
    size_t elements_within_segment = number_of_points / number_of_segments_;
    size_t idx_low = 0;
    size_t idx_high = elements_within_segment;
    cloud_segments.reserve(number_of_segments_);

    for (size_t segment_no = 0; segment_no < number_of_segments_; ++segment_no)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud_segment->points.reserve(idx_high - idx_low + 1);

        for (size_t i = idx_low; i < idx_high; ++i)
        {
            const size_t &index = sorted_indices[i];
            cloud_segment->points.push_back(cloud_points[index]);
        }
        cloud_segments.emplace_back(cloud_segment);

        // update indices
        idx_low = idx_high;
        idx_high = std::min(idx_low + elements_within_segment, number_of_points);
    }
}

// Find plane from provided ground points.
// Plane is described by (a, b, c, d) coefficients
template <typename PointT>
GroundPlane GroundSegmentation::estimatePlane(const typename pcl::PointCloud<PointT> &ground_cloud)
{
    const size_t &number_of_points = ground_cloud.points.size();

    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
    for (size_t i = 0; i < number_of_points; ++i)
    {
        const PointT &point = ground_cloud.points[i];
        points_xyz(i, 0) = point.x;
        points_xyz(i, 1) = point.y;
        points_xyz(i, 2) = point.z;
    }

    Eigen::RowVector3f centroid = points_xyz.colwise().mean();              // 1 x 3
    Eigen::MatrixX3f points_xyz_centered = points_xyz.rowwise() - centroid; // N x 3
    Eigen::Matrix3f covariance_matrix =
        (points_xyz_centered.transpose() * points_xyz_centered) / static_cast<float>(number_of_points - 1); // 3 x 3

    // Eigendecomposition of the covariance matrix - returns ordered eigenvalues in the increasing order
    Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(covariance_matrix, Eigen::DecompositionOptions::ComputeFullU);

    // Use least singular vector as normal
    Eigen::Vector3f normal_coefficients = svd_solver.matrixU().col(2);

    const float &d = -(centroid * normal_coefficients)(0, 0);
    const float &a = normal_coefficients(0, 0);
    const float &b = normal_coefficients(1, 0);
    const float &c = normal_coefficients(2, 0);

    GroundPlane plane(a, b, c, d);
    return plane;
}

// sort point cloud in the increasing order of z values
// identify points that are close to the ground level
// copy ground point candidates into seed_cloud
template <typename PointT>
void GroundSegmentation::extractInitialSeeds(const typename pcl::PointCloud<PointT> &cloud,
                                             typename pcl::PointCloud<PointT> &seed_cloud)
{
    // check the number of points in the cloud
    if (cloud.empty())
    {
        return;
    }
    const size_t &number_of_points = cloud.points.size();

    // firstly copy points into another vector for sorting
    typename std::vector<PointT> cloud_z_sorted(cloud.points.begin(), cloud.points.end());

    // sort points in the increasing z-order
    std::sort(cloud_z_sorted.begin(), cloud_z_sorted.end(),
              [&](const PointT &point_1, const PointT &point_2) -> bool { return point_1.z < point_2.z; });

    // remove outlier points (points that are presumably below ground level, with some additional offset)
    const float &negative_offset_threshold = -1.5 * sensor_height_;
    size_t lower_cutoff_index = 0;
    while (cloud_z_sorted[lower_cutoff_index].z < negative_offset_threshold)
    {
        ++lower_cutoff_index;
    }
    cloud_z_sorted.erase(cloud_z_sorted.begin(), cloud_z_sorted.begin() + lower_cutoff_index);

    // if cloud is empty, it means that seeds cannot be found, so return old seeds?
    if (cloud_z_sorted.empty())
    {
        return;
    }

    // find the average height of the lowest point representatives
    float lowest_point_representative_height = 0.0F;
    size_t number_of_estimators =
        std::min(cloud_z_sorted.size(), static_cast<size_t>(number_of_lowest_point_representative_estimators_));
    for (size_t i = 0; i < number_of_estimators; ++i)
    {
        lowest_point_representative_height += cloud_z_sorted[i].z;
    }
    lowest_point_representative_height /= number_of_lowest_point_representative_estimators_;

    // filter points that have height less that lowest_point_representative_height + initial_seed_threshold_
    const float &cutoff_height = lowest_point_representative_height + initial_seed_threshold_;
    size_t upper_cutoff_index = 0;
    for (size_t i = 0; i < cloud_z_sorted.size(); ++i)
    {
        if (cloud_z_sorted[i].z > cutoff_height)
        {
            upper_cutoff_index = i;
            break;
        }
    }

    // clear old points
    seed_cloud.clear();

    // move points from cloud_z_sorted to seed_cloud.points
    seed_cloud.points.insert(seed_cloud.points.end(), std::make_move_iterator(cloud_z_sorted.begin()),
                             std::make_move_iterator(cloud_z_sorted.begin() + upper_cutoff_index));

    // std::cout << "Number of seed points: " << seed_cloud.points.size() << std::endl;
}

// Ground segmentation for a cloud segment
template <typename PointT>
void GroundSegmentation::fitGroundPlane(const typename pcl::PointCloud<PointT> &cloud_segment,
                                        typename pcl::PointCloud<PointT> &ground_cloud_segment,
                                        typename pcl::PointCloud<PointT> &obstacle_cloud_segment)

{
    const size_t &number_of_points = cloud_segment.points.size();
    if (number_of_points == 0)
    {
        return;
    }

    // convert input_cloud to a matrix form
    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
    for (size_t i = 0; i < number_of_points; ++i)
    {
        const PointT &point = cloud_segment[i];
        points_xyz.row(i) << point.x, point.y, point.z;
    }

    // extract initial seeds (ground points)
    extractInitialSeeds(cloud_segment, ground_cloud_segment);
    obstacle_cloud_segment.reserve(number_of_points - ground_cloud_segment.size());

    // iterate for number of iterations to refine ground plane fit
    for (size_t iter_no = 0; iter_no < number_of_iterations_; ++iter_no)
    {
        if (ground_cloud_segment.empty())
        {
            break;
        }

        // estimate plane parameters a, b, c, d
        const GroundPlane &plane = estimatePlane(ground_cloud_segment);

        // clear old points
        ground_cloud_segment.clear();
        obstacle_cloud_segment.clear();

        // calculate distance from each point to the plane
        Eigen::Vector3f normal; // 3 x 1
        normal << plane.a, plane.b, plane.c;

        // ax0 + by0 + cz0 - d
        Eigen::VectorXf distances_unnormalized = (points_xyz * normal).array() - plane.d;

        // D * sqrt(a^2 + b^2 + c^2)
        const float &scaled_distance_threshold =
            distance_threshold_ * std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);

        // label points and ground and nonground
        for (size_t k = 0; k < number_of_points; ++k)
        {
            const PointT &point = cloud_segment.points[k];
            if (distances_unnormalized[k] < scaled_distance_threshold)
            {
                ground_cloud_segment.points.emplace_back(point);
            }
            else
            {
                obstacle_cloud_segment.points.emplace_back(point);
            }
        }
    }
}

// Copy segmented points into segmented cloud
// Label: 0 - ground, 1 - non-ground
inline void copySegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_segments,
                                pcl::PointCloud<pcl::PointXYZRGBI> &segmented_cloud, uint8_t label)
{
    // clear old points
    segmented_cloud.clear();

    // preallocate memory
    unsigned int number_of_elements = 0U;
    for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_segment : cloud_segments)
    {
        number_of_elements += cloud_segment->points.size();
    }
    segmented_cloud.resize(number_of_elements);

    // copy elements from each segment
    for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_segment : cloud_segments)
    {
        for (const pcl::PointXYZI &point : cloud_segment->points)
        {
            pcl::PointXYZRGBI point_cache;
            point_cache.x = point.x;
            point_cache.y = point.y;
            point_cache.z = point.z;

            lidar_processing::Color color = POINT_CLOUD_LABELS[label];
            point_cache.r = color.r;
            point_cache.g = color.g;
            point_cache.b = color.b;

            point_cache.intensity = point.intensity;

            // move cache into segmented_cloud
            segmented_cloud.points.emplace_back(std::move(point_cache));
        }
    }
    segmented_cloud.height = 1;
    segmented_cloud.width = segmented_cloud.points.size();
}

// Placed segmented cloud into segmented_cloud
void GroundSegmentation::segmentGround(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                       pcl::PointCloud<pcl::PointXYZRGBI> &ground_cloud,
                                       pcl::PointCloud<pcl::PointXYZRGBI> &obstacle_cloud)
{
    size_t number_of_points = cloud.points.size();
    if (number_of_points == 0)
    {
        return;
    }

    // partition point cloud into segments
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_segments;
    formSegments(cloud, cloud_segments);

    // iterate over segments
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ground_cloud_segments;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_cloud_segments;

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_segment : cloud_segments)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_segment =
            std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        // segment points into ground and non-ground points
        fitGroundPlane(*cloud_segment, *ground_cloud_segment, *obstacle_cloud_segment);

        // accumulate into corresponding vectors
        ground_cloud_segments.emplace_back(std::move(ground_cloud_segment));
        obstacle_cloud_segments.emplace_back(std::move(obstacle_cloud_segment));
    }

    // combine ground and non-ground points back into a single PointCloud
    copySegmentedPoints(ground_cloud_segments, ground_cloud, lidar_processing::GROUND_LABEL);
    copySegmentedPoints(obstacle_cloud_segments, obstacle_cloud, lidar_processing::OBSTACLE_LABEL);
}

} // namespace lidar_processing