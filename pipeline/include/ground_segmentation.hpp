#ifndef GROUND_SEGMENTATION
#define GROUND_SEGMENTATION

// STL
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <execution>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <vector>

// Eigen
#include <eigen3/Eigen/Dense>

// PCL
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#define DEBUG 0

namespace lidar_processing
{
enum class SegmentationLabels
{
    GROUND = 0,
    OBSTACLE = 1
};

struct GroundPlane
{
    float a, b, c, d;
    explicit GroundPlane(const float &a_, const float &b_, const float &c_, const float &d_)
        : a(a_), b(b_), c(c_), d(d_){};
    ~GroundPlane() = default;
};

struct ColorRGB
{
    float r;
    float g;
    float b;
    ColorRGB(const float &r_, const float &g_, const float &b_) : r(r_), g(g_), b(b_){};
    ColorRGB() = default;
    ~ColorRGB() = default;
};

class GroundSegmenter
{
  public:
    explicit GroundSegmenter(std::uint32_t number_of_iterations = 3, std::uint32_t number_of_planar_partitions = 1,
                             std::uint32_t number_of_lowest_point_representative_estimators = 400,
                             float sensor_height = 1.73, float distance_threshold = 0.3,
                             float initial_seed_threshold = 0.6)
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
    };

    virtual ~GroundSegmenter() = default;

    template <typename PointT>
    void segmentGround(const pcl::PointCloud<PointT> &input_cloud, pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                       pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud);

    // Setters
    void setNumberOfIterations(const std::uint32_t &number_of_iterations)
    {
        number_of_iterations_ = number_of_iterations;
    }
    void setNumberOfPlanarPartitions(const std::uint32_t &number_of_planar_partitions)
    {
        number_of_planar_partitions_ = number_of_planar_partitions;
    }
    void setNumberOfLowestPointRepresentativeEstimators(
        const std::uint32_t &number_of_lowest_point_representative_estimators)
    {
        number_of_lowest_point_representative_estimators_ = number_of_lowest_point_representative_estimators;
    }
    void setSensorHeight(const float &sensor_height)
    {
        sensor_height_ = sensor_height;
    }
    void setDistanceThreshold(const float &distance_threshold)
    {
        distance_threshold_ = distance_threshold;
    }
    void setInitialSeedThreshold(const float &initial_seed_threshold)
    {
        initial_seed_threshold_ = initial_seed_threshold;
    }

    // Getters
    const std::uint32_t &getNumberOfIterations() const
    {
        return number_of_iterations_;
    }
    const std::uint32_t &getNumberOfPlanarPartitions() const
    {
        return number_of_planar_partitions_;
    }
    const std::uint32_t &getNumberOfLowestPointRepresentativeEstimators() const
    {
        return number_of_lowest_point_representative_estimators_;
    }
    const float &getSensorHeight() const
    {
        return sensor_height_;
    }
    const float &getDistanceThreshold() const
    {
        return distance_threshold_;
    }
    const float &getInitialSeedThreshold() const
    {
        return initial_seed_threshold_;
    }

  private:
    template <typename PointT>
    void formPlanarPartitions(const pcl::PointCloud<PointT> &cloud,
                              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments);

    void extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<int> &indices);

    void fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &ground_cloud_segment,
                        pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud_segment);

    void combineSegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments,
                                const SegmentationLabels &segmentation_label,
                                pcl::PointCloud<pcl::PointXYZRGBL> &segmented_cloud) const;

  private:
    std::uint32_t number_of_iterations_;
    std::uint32_t number_of_planar_partitions_;
    std::uint32_t number_of_lowest_point_representative_estimators_;
    float sensor_height_;
    float distance_threshold_;
    float initial_seed_threshold_;
};

// Find plane from provided ground points.
// Plane equation: ax + by + cz = d
GroundPlane estimatePlane(const Eigen::MatrixXf &points_xyz)
{
    const auto number_of_points = points_xyz.rows();
    if (number_of_points < 3)
    {
        throw std::runtime_error("Cannot estimate plane parameters for less than three points");
    }

    Eigen::RowVector3f centroid = points_xyz.colwise().mean();              // 1 x 3
    Eigen::MatrixX3f points_xyz_centered = points_xyz.rowwise() - centroid; // N x 3
    Eigen::Matrix3f covariance_matrix =
        (points_xyz_centered.transpose() * points_xyz_centered) / static_cast<float>(number_of_points - 1); // 3 x 3

    // Eigendecomposition of the covariance matrix - returns ordered eigenvalues in the increasing order
    Eigen::JacobiSVD<Eigen::MatrixXf> svd_solver(covariance_matrix, Eigen::DecompositionOptions::ComputeFullU);

    // Use least singular vector as normal
    // Column two contains normal to the plane
    // See https://stackoverflow.com/questions/39370370/eigen-and-svd-to-find-best-fitting-plane-given-a-set-of-points
    Eigen::Vector3f normal_coefficients = svd_solver.matrixU().col(2);

    const float &d = -(centroid * normal_coefficients)(0, 0);
    const float &a = normal_coefficients(0, 0);
    const float &b = normal_coefficients(1, 0);
    const float &c = normal_coefficients(2, 0);

#if DEBUG
    std::cout << "Planar coefficients (a, b, c, d) = (" << a << ", " << b << ", " << c << ", " << d << ")" << std::endl;
#endif

    return GroundPlane(a, b, c, d);
}

// partitions space into multiple planar components
template <typename PointT>
void GroundSegmenter::formPlanarPartitions(const pcl::PointCloud<PointT> &cloud,
                                           std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments)
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
              [&cloud_points](const size_t &idx_1, const size_t &idx_2) -> bool {
                  return cloud_points[idx_1].x < cloud_points[idx_2].x;
              });

    // iterate over sorted indices and partition point cloud
    size_t elements_within_segment = number_of_points / number_of_planar_partitions_;
    size_t idx_low = 0;
    size_t idx_high = elements_within_segment;
    cloud_segments.reserve(number_of_planar_partitions_);

    pcl::PointXYZ point_cache;
    for (size_t segment_no = 0; segment_no < number_of_planar_partitions_; ++segment_no)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        cloud_segment->points.reserve(idx_high - idx_low + 1);

        for (size_t sorted_idx_no = idx_low; sorted_idx_no < idx_high; ++sorted_idx_no)
        {
            const auto &cloud_point = cloud_points[sorted_indices[sorted_idx_no]];

            point_cache.x = cloud_point.x;
            point_cache.y = cloud_point.y;
            point_cache.z = cloud_point.z;

            cloud_segment->points.push_back(point_cache);
        }

        cloud_segments.emplace_back(std::move(cloud_segment));

        // update indices
        idx_low = idx_high;
        idx_high = std::min(idx_low + elements_within_segment, number_of_points);
    }
}

// initial estimation of ground points
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

// Ground segmentation for a planar partition
void GroundSegmenter::fitGroundPlane(const pcl::PointCloud<pcl::PointXYZ> &cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZ> &ground_cloud_segment,
                                     pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud_segment)
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
                ground_cloud_indices.emplace_back(k);
            }
            else
            {
                obstacle_cloud_indices.emplace_back(k);
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

void GroundSegmenter::combineSegmentedPoints(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_segments,
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
        number_of_elements += cloud_segment->size();
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
        for (const auto &point : cloud_segment->points)
        {
            point_cache.x = point.x;
            point_cache.y = point.y;
            point_cache.z = point.z;

            segmented_cloud.points.emplace_back(point_cache);
        }
    }
    segmented_cloud.height = 1;
    segmented_cloud.width = segmented_cloud.points.size();
}

template <typename PointT>
void GroundSegmenter::segmentGround(const pcl::PointCloud<PointT> &input_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &ground_cloud,
                                    pcl::PointCloud<pcl::PointXYZRGBL> &obstacle_cloud)
{
    ground_cloud.clear();
    obstacle_cloud.clear();

    if (input_cloud.empty())
    {
        return;
    }

    const auto &number_of_points = input_cloud.points.size();

    // partition point cloud into segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments;
    formPlanarPartitions(input_cloud, cloud_segments);

    // iterate over segments
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ground_cloud_segments;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obstacle_cloud_segments;

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_segment : cloud_segments)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // segment points into ground and non-ground points
        fitGroundPlane(*cloud_segment, *ground_cloud_segment, *obstacle_cloud_segment);

        // accumulate into corresponding vectors
        ground_cloud_segments.emplace_back(std::move(ground_cloud_segment));
        obstacle_cloud_segments.emplace_back(std::move(obstacle_cloud_segment));
    }

    combineSegmentedPoints(ground_cloud_segments, SegmentationLabels::GROUND, ground_cloud);
    combineSegmentedPoints(obstacle_cloud_segments, SegmentationLabels::OBSTACLE, obstacle_cloud);
}

} // namespace lidar_processing

#endif // GROUND_SEGMENTATION