#include "ground_segmentation.hpp"

namespace lidar_processing
{
GroundSegmentation::GroundSegmentation(unsigned int number_of_iterations, unsigned int number_of_segments,
                                       unsigned int number_of_lowest_point_representative_estimators,
                                       float sensor_height, float distance_threshold, float initial_seed_threshold)
    : number_of_iterations_(number_of_iterations), number_of_segments_(number_of_segments),
      number_of_lowest_point_representative_estimators_(number_of_lowest_point_representative_estimators),
      sensor_height_(sensor_height), distance_threshold_(distance_threshold),
      initial_seed_threshold_(initial_seed_threshold)
{
}

// Find plane from provided ground points.
// Plane is described by (a, b, c, d) coefficients
GroundPlane GroundSegmentation::estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground_cloud)
{
    const size_t &number_of_points = ground_cloud.points.size();

    Eigen::MatrixX3f points_xyz; // N x 3
    points_xyz.resize(number_of_points);
    for (size_t i = 0; i < number_of_points; ++i)
    {
        const pcl::PointXYZI &point = ground_cloud.points[i];
        points_xyz(i, 0) = point.x;
        points_xyz(i, 1) = point.y;
        points_xyz(i, 2) = point.z;
    }

    Eigen::RowVector3f centroid = points_xyz.colwise().mean();              // 1 x 3
    Eigen::MatrixX3f points_xyz_centered = points_xyz.rowwise() - centroid; // N x 3
    Eigen::Matrix3f covariance_matrix =
        (points_xyz_centered.transpose() * points_xyz_centered) / static_cast<float>(number_of_points - 1); // 3 x 3

    // Eigendecomposition of the covariance matrix - returns ordered eigenvalues in the increasing order
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver;
    eigensolver.compute(covariance_matrix);

    Eigen::Vector3f eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix3f eigenvectors = eigensolver.eigenvectors();

    Eigen::Vector3f normal_coefficients = eigenvectors(Eigen::all, 0);
    float d = -(centroid * normal_coefficients)(0, 0);
    float a = normal_coefficients(0, 0);
    float b = normal_coefficients(1, 0);
    float c = normal_coefficients(2, 0);

    GroundPlane plane(a, b, c, d);
    return plane;
}

void GroundSegmentation::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                             pcl::PointCloud<pcl::PointXYZI> &seed_cloud)
{
    const size_t &number_of_points = cloud.points.size();
    if (number_of_points == 0)
    {
        return;
    }

    // copy points into a vector
    std::vector<std::array<float, 4>> points_z_sorted;
    points_z_sorted.reserve(number_of_points);

    for (size_t i = 0; i < number_of_points; ++i)
    {
        const pcl::PointXYZI &point = cloud.points[i];
        std::array<float, 4> point_copied = {point.x, point.y, point.z, point.intensity};
        points_z_sorted.emplace_back(point_copied);
    }

    // sort points in ascending z-order
    std::sort(points_z_sorted.begin(), points_z_sorted.end(),
              [&](const std::array<float, 4> &point_1, const std::array<float, 4> &point_2) {
                  return point_1[2] < point_2[2];
              });

    // remove outlier points (below ground points)
    float negative_offset_threshold = -1.5 * sensor_height_;
    size_t cutoff_idx = 0;
    while (points_z_sorted[cutoff_idx][2] < negative_offset_threshold)
    {
        ++cutoff_idx;
    }
    points_z_sorted.erase(points_z_sorted.begin(), points_z_sorted.begin() + cutoff_idx);

    // find the lowest point representative
    float lowest_point_representative_height = 0.0F;
    size_t number_of_points_considered =
        std::min(static_cast<size_t>(number_of_lowest_point_representative_estimators_), points_z_sorted.size());

    if (number_of_points_considered > 0)
    {
        for (size_t i = 0; i < number_of_points_considered; ++i)
        {
            lowest_point_representative_height += points_z_sorted[i][2];
        }
        lowest_point_representative_height /= number_of_points_considered;
    }
    else
    {
        return;
    }

    // filter height less that lowest_point_representative_height + initial_seed_threshold_
    seed_cloud.clear();
    const float &cutoff_height = lowest_point_representative_height + initial_seed_threshold_;
    for (const std::array<float, 4> &point_z_sorted : points_z_sorted)
    {
        if (point_z_sorted[2] > cutoff_height)
        {
            break;
        }
        const pcl::PointXYZI &point{point_z_sorted[0], point_z_sorted[1], point_z_sorted[2], point_z_sorted[3]};
        seed_cloud.points.emplace_back(point);
    }
}

} // namespace lidar_processing