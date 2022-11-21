#include "ground_segmentation.hpp"

namespace lidar_processing
{
// Find plane from provided ground points.
// Plane is described by (a, b, c, d) coefficients
GroundPlane GroundSegmentation::estimatePlane(const pcl::PointCloud<pcl::PointXYZI> &ground_cloud)
{
    const size_t &number_of_points = ground_cloud.points.size();

    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
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

// Placed segmented cloud into segmented_cloud
void GroundSegmentation::segmentGround(const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                                       pcl::PointCloud<pcl::PointXYZIL> &segmented_cloud)
{
    // convert input_cloud to matrix
    size_t number_of_points = input_cloud.points.size();
    segmented_cloud.points.reserve(number_of_points);
    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
    for (const auto &cloud_point : input_cloud)
    {
        points_xyz(0, 0) = cloud_point.x;
        points_xyz(0, 1) = cloud_point.y;
        points_xyz(0, 2) = cloud_point.z;
    }

    // extract seeds
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    extractInitialSeeds(input_cloud, *ground_cloud);
    std::vector<size_t> ground_indices;
    std::vector<size_t> nonground_indices;
    ground_indices.reserve(ground_cloud->points.size());
    nonground_indices.reserve(ground_indices.size());

    // iterate for number of iterations to refine ground plane fit
    for (size_t iter_no = 0; iter_no < number_of_iterations_; ++iter_no)
    {
        // estimate plane parameters a, b, c, d
        const GroundPlane &plane = estimatePlane(*ground_cloud);

        ground_cloud->points.clear();
        ground_indices.clear();
        nonground_indices.clear();

        // calculate distance from each point to plane
        Eigen::Vector3f normal; // 3 x 1
        normal(0, 0) = plane.a;
        normal(1, 0) = plane.b;
        normal(2, 0) = plane.c;

        // |ax0 + by0 + cz0 - d|
        Eigen::VectorXf distances = Eigen::abs((points_xyz * normal).array() - plane.d);

        // D * sqrt(a^2 + b^2 + c^2)
        float scaled_distance_threshold =
            distance_threshold_ * std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);

        // label points and ground and nonground
        for (size_t k = 0; k < distances.rows(); ++k)
        {
            pcl::PointXYZI point = input_cloud.points[k];
            if (distances(k, 0) < scaled_distance_threshold)
            {
                ground_cloud->points.emplace_back(point);
                ground_indices.emplace_back(k);
            }
            else
            {
                nonground_indices.emplace_back(k);
            }
        }
    }

    // free some memory
    ground_cloud.reset();

    // copy ground points into output cloud and add label = 0
    for (const size_t &ground_index : ground_indices)
    {
        pcl::PointXYZIL point;
        const pcl::PointXYZI &input_point = input_cloud.points[ground_index];
        point.x = input_point.x;
        point.y = input_point.y;
        point.z = input_point.z;
        point.intensity = input_point.intensity;
        point.label = 0;
        segmented_cloud.emplace_back(point);
    }

    // copy nonground points into output cloud and add label = 1
    for (const size_t &nonground_index : nonground_indices)
    {
        pcl::PointXYZIL point;
        const pcl::PointXYZI &input_point = input_cloud.points[nonground_index];
        point.x = input_point.x;
        point.y = input_point.y;
        point.z = input_point.z;
        point.intensity = input_point.intensity;
        point.label = 1;
        segmented_cloud.emplace_back(point);
    }

    std::cout << "Number of ground points: " << ground_indices.size() << std::endl;
    std::cout << "Number of nonground points: " << nonground_indices.size() << std::endl;
}

} // namespace lidar_processing