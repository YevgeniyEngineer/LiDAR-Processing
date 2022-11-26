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

    std::cout << "Number of seed points: " << seed_cloud.points.size() << std::endl;
}

// // Partition point cloud into multiple segments
// // Uses random index partitioning scheme
// void GroundSegmentation::formSegments(const pcl::PointCloud<pcl::PointXYZI> &cloud,
//                                       std::vector<pcl::PointCloud<pcl::PointXYZIIDX>::Ptr> &cloud_segments)
// {
//     cloud_segments.clear();
//     const size_t &number_of_points = cloud.points.size();

//     // create indices
//     std::vector<size_t> indices;
//     indices.reserve(number_of_points);
//     for (size_t i = 0; i < number_of_points; ++i)
//     {
//         indices.emplace_back(i);
//     }

//     // select random cloud indices
//     std::random_device random_seed_generator;
//     std::mt19937 pseudo_random_number_generator(random_seed_generator());
//     std::shuffle(indices.begin(), indices.end(), pseudo_random_number_generator);

//     // create cloud segments based on random indices
//     size_t elements_within_segment = number_of_points / number_of_segments_;
//     size_t idx_low = 0;
//     size_t idx_high = elements_within_segment;
//     cloud_segments.reserve(number_of_segments_);
//     for (size_t segment_no = 0; segment_no < number_of_segments_; ++segment_no)
//     {
//         pcl::PointCloud<pcl::PointXYZIIDX>::Ptr cloud_segment =
//         std::make_shared<pcl::PointCloud<pcl::PointXYZIIDX>>(); for (size_t i = idx_low; i < idx_high; ++i)
//         {
//             const size_t &index = indices[i];
//             const pcl::PointXYZI &cloud_point = cloud.points[index];

//             pcl::PointXYZIIDX segment_point;
//             segment_point.x = cloud_point.x;
//             segment_point.y = cloud_point.y;
//             segment_point.z = cloud_point.z;
//             segment_point.intensity = cloud_point.intensity;
//             segment_point.index = index;

//             cloud_segment->points.emplace_back(segment_point);
//         }
//         cloud_segments.emplace_back(cloud_segment);

//         // update indices
//         idx_low = idx_high;
//         idx_high = std::min(idx_low + elements_within_segment, number_of_points);
//     }
// }

// Partition points into random segments
// Partitions along x-axis, that is the direction of forward vehicle movement
void GroundSegmentation::formSegments(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                      std::vector<pcl::PointCloud<pcl::PointXYZIIDX>::Ptr> &cloud_segments)
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
        pcl::PointCloud<pcl::PointXYZIIDX>::Ptr cloud_segment = std::make_shared<pcl::PointCloud<pcl::PointXYZIIDX>>();
        cloud_segment->points.reserve(idx_high - idx_low + 1);

        for (size_t i = idx_low; i < idx_high; ++i)
        {
            const size_t &index = sorted_indices[i];
            const pcl::PointXYZI &cloud_point = cloud_points[index];

            pcl::PointXYZIIDX segment_point;
            segment_point.x = cloud_point.x;
            segment_point.y = cloud_point.y;
            segment_point.z = cloud_point.z;
            segment_point.intensity = cloud_point.intensity;
            segment_point.index = index;

            cloud_segment->points.emplace_back(segment_point);
        }
        cloud_segments.emplace_back(cloud_segment);

        // update indices
        idx_low = idx_high;
        idx_high = std::min(idx_low + elements_within_segment, number_of_points);
    }
}

// Placed segmented cloud into segmented_cloud
void GroundSegmentation::segmentGround(const pcl::PointCloud<pcl::PointXYZI> &input_cloud,
                                       pcl::PointCloud<pcl::PointXYZIL> &segmented_cloud)
{
    size_t number_of_points = input_cloud.points.size();
    if (number_of_points == 0)
    {
        return;
    }
    segmented_cloud.points.reserve(number_of_points);

    // partition point cloud into segments
    std::vector<pcl::PointCloud<pcl::PointXYZIIDX>::Ptr> cloud_segments;
    formSegments(input_cloud, cloud_segments);

    // convert input_cloud to matrix
    Eigen::MatrixXf points_xyz(number_of_points, 3); // N x 3
    for (size_t i = 0; i < number_of_points; ++i)
    {
        pcl::PointXYZI pt = input_cloud[i];
        points_xyz.row(i) << pt.x, pt.y, pt.z;
    }

    // extract seeds
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr non_ground_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    extractInitialSeeds(input_cloud, *ground_cloud);

    // iterate for number of iterations to refine ground plane fit
    std::vector<size_t> ground_indices;
    std::vector<size_t> non_ground_indices;
    ground_indices.reserve(ground_cloud->points.size());
    non_ground_indices.reserve(ground_indices.size());
    for (size_t iter_no = 0; iter_no < number_of_iterations_; ++iter_no)
    {
        if (ground_cloud->points.empty())
        {
            break;
        }

        // estimate plane parameters a, b, c, d
        const GroundPlane &plane = estimatePlane(*ground_cloud);

        // clear old points
        ground_cloud->points.clear();
        ground_indices.clear();
        non_ground_indices.clear();

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
            const pcl::PointXYZI &point = input_cloud.points[k];
            if (distances_unnormalized[k] < scaled_distance_threshold)
            {
                ground_cloud->points.emplace_back(point);
                ground_indices.emplace_back(k);
            }
            else
            {
                non_ground_indices.emplace_back(k);
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
    for (const size_t &nonground_index : non_ground_indices)
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
    std::cout << "Number of nonground points: " << non_ground_indices.size() << std::endl;
}

} // namespace lidar_processing