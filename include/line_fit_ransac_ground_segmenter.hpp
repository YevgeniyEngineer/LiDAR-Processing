#ifndef LINE_FIT_RANSAC_GROUND_SEGMENTER
#define LINE_FIT_RANSAC_GROUND_SEGMENTER

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <execution>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <ratio>
#include <thread>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
static float atan2_approx(const float y, const float x) noexcept
{
    const float ax = std::fabs(x);
    const float ay = std::fabs(y);
    const float mx = std::max(ay, ax);
    const float mn = std::min(ay, ax);
    const float a = mn / mx;
    /* Minimax polynomial approximation to atan(a) on [0,1] */
    const float s = a * a;
    const float c = s * a;
    const float q = s * s;
    float r = 0.024840285f * q + 0.18681418f;
    const float t = -0.094097948f * q - 0.33213072f;
    r = r * s + t;
    r = r * c + a;
    /* Map to full circle */
    if (ay > ax)
    {
        r = 1.57079637f - r;
    }
    if (x < 0)
    {
        r = 3.14159274f - r;
    }
    if (y < 0)
    {
        r = -r;
    }
    return r;
}

class LineFitRANSACGroundSegmenter final
{
  public:
    static constexpr float SLICE_RESOLUTION_DEG{0.5F};
    static constexpr std::uint32_t APPROXIMATE_MAX_POINTS_IN_INPUT_CLOUD{300'000U};

    static constexpr float FLOAT_EPSILON{1e-5F};
    static constexpr std::uint32_t NUMBER_OF_SLICES{
        static_cast<std::uint32_t>((360.0F / SLICE_RESOLUTION_DEG) + FLOAT_EPSILON)};

    static constexpr std::uint32_t APPROXIMATE_POINTS_PER_SLICE{static_cast<std::uint32_t>(
        static_cast<double>(APPROXIMATE_MAX_POINTS_IN_INPUT_CLOUD) / static_cast<double>(NUMBER_OF_SLICES))};

    enum class SegmentationLabel : std::uint8_t
    {
        UNCLASSIFIED = 0U,
        GROUND,
        OBSTACLE
    };

    struct ProcessingPoint final
    {
        float x;
        float y;

        // Polar distance
        float distance;

        // Effectively z coordinate
        float height;

        // Index mapping to the input point cloud
        std::uint32_t index;

        // Segmentation label
        SegmentationLabel label;

        ProcessingPoint() = default;

        ProcessingPoint(float x, float y, float distance, float height, std::uint32_t index, SegmentationLabel label)
            : x{x}, y{y}, distance{distance}, height{height}, index{index}, label{label}
        {
        }
    };

    LineFitRANSACGroundSegmenter()
    {
        // Reserve memory for slices
        for (auto &slice : slices_)
        {
            slice.reserve(APPROXIMATE_POINTS_PER_SLICE);
        }
    }

    ~LineFitRANSACGroundSegmenter()
    {
    }

    template <typename CloudT> void setInputCloud(const CloudT &input_cloud)
    {
        static constexpr float RAD_TO_DEG = 180.0F / M_PIf32;
        static constexpr float CONST = 1.0F / 360.0F;

        // Clear old points
        for (auto &slice : slices_)
        {
            slice.clear();
        }

        const auto t1 = std::chrono::steady_clock::now();

        // Form new slices
        for (std::uint32_t point_index = 0U; point_index < input_cloud.size(); ++point_index)
        {
            const auto &point = input_cloud[point_index];

            // Convert azimuth angle to degrees and shift range to [0, 360)
            // const float azimuth_rad = std::atan2(point.y, point.x);
            const float azimuth_rad = atan2_approx(point.y, point.x);

            // const float azimuth_deg = std::fmod((azimuth_rad * RAD_TO_DEG) + 360.0F, 360.0F);
            float azimuth_deg = azimuth_rad * RAD_TO_DEG;
            azimuth_deg -= 360.F * std::floor(azimuth_deg * CONST);

            // Determine slice index using floor division
            const std::uint32_t slice_index = static_cast<std::uint32_t>(azimuth_deg / SLICE_RESOLUTION_DEG);

            // Calculate distance
            const float distance = std::sqrt((point.x * point.x) + (point.y * point.y));

            // Add point to the slice
            slices_[slice_index].emplace_back(point.x, point.y, distance, point.z, point_index,
                                              SegmentationLabel::UNCLASSIFIED);
        }

        const auto t2 = std::chrono::steady_clock::now();

        // Sort points in ascending radial order in each slice
        std::for_each(std::execution::par, slices_.begin(), slices_.end(), [](auto &slice) noexcept -> void {
            std::sort(slice.begin(), slice.end(),
                      [](const auto &p1, const auto &p2) noexcept -> bool { return (p1.distance < p2.distance); });
        });

        const auto t3 = std::chrono::steady_clock::now();

        // std::cout << "Arrangement time [microsec]: "
        //           << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << std::endl;

        // std::cout << "Sort time [microsec]: " << std::chrono::duration_cast<std::chrono::microseconds>(t3 -
        // t2).count()
        //           << std::endl;
    }

    void segment(pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud)
    {
        // Clear cache
        ground_cloud.clear();
        obstacle_cloud.clear();

        // Perform segmentation
        // TODO: Function call here

        // Copy classified points
        for (const auto &slice : slices_)
        {
            for (const auto &point : slice)
            {
                switch (point.label)
                {
                case SegmentationLabel::GROUND: {
                    ground_cloud.emplace_back(point.x, point.y, point.height);
                    break;
                }
                case SegmentationLabel::OBSTACLE: {
                    obstacle_cloud.emplace_back(point.x, point.y, point.height);
                    break;
                }
                case SegmentationLabel::UNCLASSIFIED: {
                    break;
                }
                default: {
                    break;
                }
                }
            }
        }

        ground_cloud.is_dense = true;
        obstacle_cloud.is_dense = true;

        ground_cloud.height = 1;
        obstacle_cloud.height = 1;

        ground_cloud.width = ground_cloud.points.size();
        obstacle_cloud.width = obstacle_cloud.points.size();
    }

  private:
    std::array<std::vector<ProcessingPoint>, NUMBER_OF_SLICES> slices_;

    float orthogonal_distance_threshold_{0.1F};
    std::uint8_t number_of_threads_{8U};

    void constrainedLineRANSAC(const std::vector<ProcessingPoint> &slice)
    {
    }

    void segmentImpl()
    {
    }
};
} // namespace lidar_processing

#endif // LINE_FIT_RANSAC_GROUND_SEGMENTER
