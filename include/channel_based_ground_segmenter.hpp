#ifndef CHANNEL_BASED_GROUND_SEGMENTER
#define CHANNEL_BASED_GROUND_SEGMENTER

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_processing
{
enum class SegmentationLabel : std::uint32_t
{
    DOUBT = 0U,
    GROUND,
    OBSTACLE
};

struct PointXYZIL final
{
    float x;
    float y;
    float z;
    std::uint32_t index;
    SegmentationLabel label;
};

class ChannelBasedGroundSegmenter final
{
  public:
    // 1 degree increments
    // static constexpr float CHANNEL_RESOLUTION_DEG{1.0F};
    // static constexpr std::uint32_t NUMBER_OF_CHANNELS{360U};

    // 2 degree increments
    static constexpr float CHANNEL_RESOLUTION_DEG{2.0F};
    static constexpr std::uint32_t NUMBER_OF_CHANNELS{180U};

    // 4 degree increments
    // static constexpr float CHANNEL_RESOLUTION_DEG{4.0F};
    // static constexpr std::uint32_t NUMBER_OF_CHANNELS{90U};

    static constexpr float CELL_RESOLUTION_M{1.5F};

    struct CellBoundary final
    {
        std::uint32_t start;
        std::uint32_t stop;
    };

    ChannelBasedGroundSegmenter(const ChannelBasedGroundSegmenter &other) = delete;
    ChannelBasedGroundSegmenter &operator=(const ChannelBasedGroundSegmenter &other) = delete;
    ChannelBasedGroundSegmenter(ChannelBasedGroundSegmenter &&other) = delete;
    ChannelBasedGroundSegmenter &operator=(ChannelBasedGroundSegmenter &&other) = delete;

    ChannelBasedGroundSegmenter(float vehicle_height = 1.73F, float threshold_distance_m = 70.0F,
                                float threshold_gradient_deg = 20.0F, float threshold_ground_height_m = 0.2F,
                                float threshold_height_m = 0.5F, float height_difference_parameter_m = 0.1F)
        : vehicle_height_{vehicle_height}, threshold_distance_m_{threshold_distance_m},
          threshold_gradient_rad_{threshold_gradient_deg * M_PIf32 / 180.0F},
          threshold_ground_height_m_{threshold_ground_height_m}, threshold_height_m_{threshold_height_m},
          height_difference_parameter_m_{height_difference_parameter_m}
    {
    }

    ~ChannelBasedGroundSegmenter()
    {
    }

    template <typename CloudT> void setInputCloud(const CloudT &input_cloud)
    {
        input_cloud_.clear();
        input_cloud_.reserve(input_cloud.size());
        for (std::uint32_t point_index = 0U; point_index < input_cloud.size(); ++point_index)
        {
            const auto &point = input_cloud[point_index];
            input_cloud_.push_back(PointXYZIL{point.x, point.y, point.z, point_index, SegmentationLabel::DOUBT});
        }

        partitionIntoChannels();
    }

    void segment(pcl::PointCloud<pcl::PointXYZ> &ground_cloud, pcl::PointCloud<pcl::PointXYZ> &obstacle_cloud)
    {
        ground_cloud.clear();
        obstacle_cloud.clear();

        segmentChannels();

        for (const auto &channel : channel_points_)
        {
            for (const auto &point : channel)
            {
                if (point.label == SegmentationLabel::GROUND)
                {
                    ground_cloud.emplace_back(point.x, point.y, point.z);
                }
                else if (point.label == SegmentationLabel::OBSTACLE)
                {
                    obstacle_cloud.emplace_back(point.x, point.y, point.z);
                }
                else
                {
                    // Doubt point
                    // std::cout << "Found doubt point.\n";
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
    const float vehicle_height_;
    const float threshold_distance_m_;
    const float threshold_gradient_rad_;
    const float threshold_ground_height_m_;
    const float threshold_height_m_;
    const float height_difference_parameter_m_;

    std::vector<PointXYZIL> input_cloud_;
    std::array<std::vector<PointXYZIL>, NUMBER_OF_CHANNELS> channel_points_;
    std::array<std::vector<CellBoundary>, NUMBER_OF_CHANNELS> cell_indices_;

    void partitionIntoChannels()
    {
        // Clear old points
        for (auto &channel : channel_points_)
        {
            channel.clear();
        }
        for (auto &cell_channel : cell_indices_)
        {
            cell_channel.clear();
        }

        // Form new channels
        for (const auto &point : input_cloud_)
        {
            const float azimuth_rad = std::atan2(point.y, point.x);
            // Convert azimuth angle to degrees and shift range to [0, 360)
            const float azimuth_deg = std::max(std::fmod(((azimuth_rad * 180.0F) / M_PIf32) + 360.0F, 360.0F), 0.0F);
            // Determine channel index using floor division
            const std::uint32_t channel_index = static_cast<std::uint32_t>(azimuth_deg / CHANNEL_RESOLUTION_DEG);
            channel_points_[channel_index].push_back(point);
        }
    }

    bool checkNoise(const PointXYZIL &point)
    {
        return false;
    }

    void checkGround(PointXYZIL &curr, const PointXYZIL &prev, const PointXYZIL &ground)
    {
        curr.label = SegmentationLabel::DOUBT;

        // Calculate squared distances for current and previous points
        const float curr_dist_squared = (curr.x * curr.x) + (curr.y * curr.y);
        const float prev_dist_squared = (prev.x * prev.x) + (prev.y * prev.y);

        // Rule 1: Expected distance (current point farther than the last ground point)
        if (curr_dist_squared <= prev_dist_squared)
        {
            return; // Early exit if Rule 1 is not satisfied
        }

        // Rule 2: Height reduction (current height < previous height)
        if (curr.z >= prev.z)
        {
            return; // Early exit if Rule 2 is not satisfied
        }

        // Rule 3: Similar height to last ground point
        const float height_diff_to_last_ground = curr.z - ground.z;
        if (height_diff_to_last_ground >= threshold_ground_height_m_)
        {
            return; // Early exit if Rule 3 is not satisfied
        }

        // If all three rules are satisfied, set label to GROUND
        curr.label = SegmentationLabel::GROUND;
    }

    void checkObstacle(PointXYZIL &curr, const PointXYZIL &prev, const PointXYZIL &ground)
    {
        curr.label = SegmentationLabel::DOUBT;

        // Check height difference w.r.t. ground
        const float ground_height_diff_m = curr.z - ground.z;
        if (ground_height_diff_m <= threshold_ground_height_m_)
        {
            return;
        }
        else
        {
            curr.label = SegmentationLabel::OBSTACLE;
            return;
        }

        // Check gradient
        const float dx = curr.x - prev.x;
        const float dy = curr.y - prev.y;
        const float dz = curr.z - prev.z;
        const float distance_m_squared = dx * dx + dy * dy;
        const float distance_m = std::sqrt(distance_m_squared);
        const float gradient_rad = std::atan(dz / distance_m);
        if (gradient_rad > threshold_gradient_rad_)
        {
            curr.label = SegmentationLabel::OBSTACLE;
            return;
        }

        // Check abnormal distance
        const float curr_dist_squared = (curr.x * curr.x) + (curr.y * curr.y);
        const float prev_dist_squared = (prev.x * prev.x) + (prev.y * prev.y);
        if (curr_dist_squared < prev_dist_squared)
        {
            curr.label = SegmentationLabel::OBSTACLE;
            return;
        }
    }

    void checkBoth(PointXYZIL &curr, const PointXYZIL &prev, const PointXYZIL &ground)
    {
        checkGround(curr, prev, ground);
        if (curr.label != SegmentationLabel::GROUND)
        {
            checkObstacle(curr, prev, ground);
        }
    }

    void correctDoubtPoints(std::vector<PointXYZIL> &channel, std::uint32_t last_index, SegmentationLabel label)
    {
        for (std::int32_t j = last_index - 1; j >= 0; --j)
        {
            auto &point_label = channel[static_cast<std::uint32_t>(j)].label;
            if (point_label == SegmentationLabel::DOUBT)
            {
                point_label = label;
            }
            else
            {
                break;
            }
        }
    }

    void segmentChannel(std::vector<PointXYZIL> &channel)
    {
        if (channel.empty())
        {
            return;
        }

        PointXYZIL ground{0.0F, 0.0F, 0.0F, 0, SegmentationLabel::GROUND};

        const auto *ground_ptr = &ground;
        const PointXYZIL *prev_ptr = &ground;

        for (std::uint32_t i = 0U; i < channel.size(); ++i)
        {
            auto &curr = channel[i];
            if (checkNoise(curr))
            {
                continue;
            }

            switch (prev_ptr->label)
            {
            case SegmentationLabel::GROUND: {
                checkObstacle(curr, *prev_ptr, *ground_ptr);
                break;
            }
            case SegmentationLabel::OBSTACLE: {
                checkGround(curr, *prev_ptr, *ground_ptr);
                break;
            }
            case SegmentationLabel::DOUBT: {
                checkBoth(curr, *prev_ptr, *ground_ptr);
                if (curr.label != SegmentationLabel::DOUBT)
                {
                    correctDoubtPoints(channel, i, curr.label);
                }
                break;
            }
            default: {
                break;
            }
            }

            prev_ptr = &curr;

            if (curr.label == SegmentationLabel::GROUND)
            {
                ground_ptr = &curr;
            }
        }
    }

    void segmentChannels()
    {
        for (auto &channel : channel_points_)
        {
            segmentChannel(channel);
        }
    }

    void channelBasedMarkovRandomFieldAndLoopyBeliefPropagation()
    {
        // https://nbviewer.org/github/krashkov/Belief-Propagation/blob/master/4-ImplementationBP.ipynb
        // https://nghiaho.com/?page_id=1366
    }
};
} // namespace lidar_processing

#endif // CHANNEL_BASED_GROUND_SEGMENTER