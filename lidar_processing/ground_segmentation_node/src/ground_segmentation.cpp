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
void GroundSegmentation::extractInitialSeeds(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                             pcl::PointCloud<pcl::PointXYZI> &seed_cloud)
{
    // check the number of points in the cloud
    if (cloud.empty())
    {
        return;
    }
    const size_t &number_of_points = cloud.points.size();

    // firstly copy points into another vector for sorting
    std::vector<pcl::PointXYZI> cloud_z_sorted(cloud.points.begin(), cloud.points.end());

    // sort points in the increasing z-order
    std::sort(
        cloud_z_sorted.begin(), cloud_z_sorted.end(),
        [&](const pcl::PointXYZI &point_1, const pcl::PointXYZI &point_2) -> bool { return point_1.z < point_2.z; });

    // remove outlier points (points that are presumably below ground level, with some additional offset)
    const float &negative_offset_threshold = -1.5 * sensor_height_;
    size_t lower_cutoff_index = 0;
    while (cloud_z_sorted[lower_cutoff_index].z < negative_offset_threshold)
    {
        ++lower_cutoff_index;
    }

    // find the average height of the lowest point representatives
    // anything higher that positive_offset_threshold will not be considered as ground points
    const float &positive_offset_threshold = 0.35F;
    float lowest_point_representative_height = 0.0F;
    size_t idx = lower_cutoff_index;
    while (cloud_z_sorted[idx].z < positive_offset_threshold &&
           idx != number_of_lowest_point_representative_estimators_)
    {
        ++idx;
        lowest_point_representative_height += cloud_z_sorted[idx].z;
    }
    lowest_point_representative_height /= static_cast<float>(idx - lower_cutoff_index + 1);

    // filter points that have height less that lowest_point_representative_height + initial_seed_threshold_
    const float &cutoff_height = lowest_point_representative_height + initial_seed_threshold_;
    size_t upper_cutoff_index = cloud_z_sorted.size();
    for (size_t i = lower_cutoff_index; i < cloud_z_sorted.size(); ++i)
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
    seed_cloud.points.insert(seed_cloud.points.end(),
                             std::make_move_iterator(cloud_z_sorted.begin() + lower_cutoff_index),
                             std::make_move_iterator(cloud_z_sorted.begin() + upper_cutoff_index));

    std::cout << "Number of seed points: " << seed_cloud.points.size() << std::endl;
}

// Placed segmented cloud into segmented_cloud
void GroundSegmentation::segmentGround(const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                                       pcl::PointCloud<pcl::PointXYZIL> &segmented_cloud)
{

    // for (const auto &pt : input_cloud)
    // {
    //     std::cout << pt.z << std::endl;
    // }

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

    // iterate for number of iterations to refine ground plane fit
    std::vector<size_t> ground_indices;
    std::vector<size_t> nonground_indices;
    ground_indices.reserve(ground_cloud->points.size());
    nonground_indices.reserve(ground_indices.size());
    for (size_t iter_no = 0; iter_no < number_of_iterations_; ++iter_no)
    {
        if (ground_cloud->points.empty())
        {
            break;
        }

        // estimate plane parameters a, b, c, d
        const GroundPlane &plane = estimatePlane(*ground_cloud);
        std::cout << "Plane: a = " << plane.a << ", b = " << plane.b << ", c = " << plane.c << ", d = " << plane.d
                  << std::endl;

        // clear old points
        ground_cloud->points.clear();
        ground_indices.clear();
        nonground_indices.clear();

        // calculate distance from each point to the plane
        Eigen::Vector3f normal; // 3 x 1
        normal(0, 0) = plane.a;
        normal(1, 0) = plane.b;
        normal(2, 0) = plane.c;

        // ax0 + by0 + cz0 - d
        Eigen::VectorXf distances = (points_xyz * normal).array() - plane.d;

        // D * sqrt(a^2 + b^2 + c^2)
        const float &scaled_distance_threshold =
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