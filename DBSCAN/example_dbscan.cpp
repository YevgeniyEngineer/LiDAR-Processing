#include "dbscan.hpp"           // DBSCAN
#include "dbscan_nanoflann.hpp" // DBSCAN_NANOFLANN
#include "point_struct.hpp"     // PointCloud
#include <chrono>               // std::chrono
#include <csignal>              // std::signal
#include <iostream>             // std::cout
#include <memory>               // std::unique_ptr
#include <random>               // std::random_device
#include <variant>              // std::variant

using namespace clustering;

// Constants
namespace configuration_parameters
{
constexpr static int NUMBER_OF_DIMENSIONS = 3;
constexpr static int NUMBER_OF_POINTS = 100'000;
constexpr static int NUMBER_OF_ITERATIONS = 100;
} // namespace configuration_parameters

namespace dbscan_parameters
{
constexpr static double NEAREST_NEIGHBOR_PROXIMITY = 0.1;
constexpr static std::int32_t MINIMUM_POINTS_TO_FORM_CLUSTER = 5;
} // namespace dbscan_parameters

using CoordinateType = double;
using PointCloudType = PointCloud<CoordinateType, configuration_parameters::NUMBER_OF_DIMENSIONS>;

using DBSCAN_Type = clustering::dbscan::DBSCAN<CoordinateType, configuration_parameters::NUMBER_OF_DIMENSIONS>;

using DBSCAN_NANOFLANN_Type =
    clustering::dbscan_nanoflann::DBSCAN_NANOFLANN<CoordinateType, configuration_parameters::NUMBER_OF_DIMENSIONS>;

using VariantType = std::variant<DBSCAN_Type, DBSCAN_NANOFLANN_Type>;

/// @brief Generate and return random 3D points
static inline void generateRandomData(PointCloudType &point_cloud)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<CoordinateType> dist(-10.0, 10.0);

    point_cloud.points.clear();
    point_cloud.points.reserve(configuration_parameters::NUMBER_OF_POINTS);
    for (auto i = 0; i < configuration_parameters::NUMBER_OF_POINTS; ++i)
    {
        std::array<CoordinateType, configuration_parameters::NUMBER_OF_DIMENSIONS> point_cache;
        for (auto j = 0; j < configuration_parameters::NUMBER_OF_DIMENSIONS; ++j)
        {
            point_cache[j] = dist(gen);
        }
        point_cloud.points.emplace_back(point_cache);
    }
}

struct VariantVisitor
{
    void operator()(DBSCAN_Type &dbscan)
    {
        dbscan.formClusters();
    }

    void operator()(DBSCAN_NANOFLANN_Type &dbscan_nanoflann)
    {
        dbscan_nanoflann.formClusters();
    }
};

static inline double runAndTimeExecution(VariantType &dbscan)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::visit(VariantVisitor{}, dbscan);

    auto stop_time = std::chrono::high_resolution_clock::now();
    auto elapsed = static_cast<double>((stop_time - start_time).count()) / 1e9;
    std::cout << "Elapsed time: " << elapsed << " seconds\n";

    return elapsed;
}

namespace
{
volatile std::sig_atomic_t signal_status;
}

int main(int argc, const char **const argv)
{
    // Install signal handler
    std::signal(SIGINT, [](int signal) {
        signal_status = signal;
        std::cout << "Keyboard interrupt\n";
        std::exit(signal);
    });

    // Generate point cloud data
    PointCloudType point_cloud;
    generateRandomData(point_cloud);

    // Run several iterations of DBSCAN
    double average_time_dbscan = 0.0;
    for (std::int32_t i = 0; i < configuration_parameters::NUMBER_OF_ITERATIONS; ++i)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        VariantType dbscan_variant(std::in_place_index_t<0>{}, dbscan_parameters::NEAREST_NEIGHBOR_PROXIMITY,
                                   dbscan_parameters::MINIMUM_POINTS_TO_FORM_CLUSTER, point_cloud);

        average_time_dbscan += runAndTimeExecution(dbscan_variant);
    }

    average_time_dbscan /= configuration_parameters::NUMBER_OF_ITERATIONS;
    std::cout << "Average time per loop (DBSCAN): " << average_time_dbscan << " seconds\n";

    // Run several iterations of DBSCAN_NANOFLANN
    double average_time_dbscan_nanoflann = 0.0;
    for (std::int32_t i = 0; i < configuration_parameters::NUMBER_OF_ITERATIONS; ++i)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        VariantType dbscan_variant(std::in_place_index_t<1>{}, dbscan_parameters::NEAREST_NEIGHBOR_PROXIMITY,
                                   dbscan_parameters::MINIMUM_POINTS_TO_FORM_CLUSTER, point_cloud);

        average_time_dbscan_nanoflann += runAndTimeExecution(dbscan_variant);
    }

    average_time_dbscan_nanoflann /= configuration_parameters::NUMBER_OF_ITERATIONS;
    std::cout << "Average time per loop (DBSCAN_NANOFLANN): " << average_time_dbscan_nanoflann << " seconds\n";

    return EXIT_SUCCESS;
}