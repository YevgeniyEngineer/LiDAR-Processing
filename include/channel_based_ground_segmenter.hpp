#ifndef CHANNEL_BASED_GROUND_SEGMENTER
#define CHANNEL_BASED_GROUND_SEGMENTER

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <execution>
#include <iostream>
#include <limits>
#include <unordered_map>
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

struct GraphNode final
{
    PointXYZIL point;
    SegmentationLabel label;
    float unary_potential;
};

struct GraphCell final
{
    // Points that fall into this cell
    std::vector<PointXYZIL> points;

    // The label assigned to this cell
    SegmentationLabel label;

    // Potential based on the initial segmentation
    float unary_potential;
};

struct GraphEdge final
{
    // Index of the start node
    std::uint32_t node_1_index;

    // Index of the end node
    std::uint32_t node_2_index;

    // Potential between two adjacent nodes
    float pairwise_potential;
};

struct Message final
{
    float to_ground;
    float to_obstacle;
};

class ChannelBasedGroundSegmenter final
{
  public:
    // 0.5 degree increments
    static constexpr float CHANNEL_RESOLUTION_DEG{0.5F};
    static constexpr std::uint32_t NUMBER_OF_CHANNELS{720U};

    // 1 degree increments
    // static constexpr float CHANNEL_RESOLUTION_DEG{1.0F};
    // static constexpr std::uint32_t NUMBER_OF_CHANNELS{360U};

    // 2 degree increments
    // static constexpr float CHANNEL_RESOLUTION_DEG{2.0F};
    // static constexpr std::uint32_t NUMBER_OF_CHANNELS{180U};

    // 4 degree increments
    // static constexpr float CHANNEL_RESOLUTION_DEG{4.0F};
    // static constexpr std::uint32_t NUMBER_OF_CHANNELS{90U};

    static constexpr float CELL_RESOLUTION_M{1.5F};
    static constexpr std::uint32_t NUMBER_OF_CELLS{200U};

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
                                float threshold_gradient_deg = 20.0F, float threshold_ground_height_m = 0.15F,
                                float threshold_height_m = 0.5F, float height_difference_parameter_m = 0.1F)
        : vehicle_height_{vehicle_height}, threshold_distance_m_{threshold_distance_m},
          threshold_gradient_rad_{threshold_gradient_deg * M_PIf32 / 180.0F},
          threshold_ground_height_m_{threshold_ground_height_m}, threshold_height_m_{threshold_height_m},
          height_difference_parameter_m_{height_difference_parameter_m}
    {
        static constexpr std::uint32_t APPROXIMATE_MAX_PTS_IN_CHANNEL{1500U};
        static constexpr std::uint32_t APPROXIMATE_MAX_PTS_IN_CELL{100U};

        for (auto &channel : polar_grid_)
        {
            for (auto &cell : channel)
            {
                cell.reserve(APPROXIMATE_MAX_PTS_IN_CELL);
            }
        }

        for (auto &channel : channel_points_)
        {
            channel.reserve(APPROXIMATE_MAX_PTS_IN_CHANNEL);
        }

        graph_cells_.reserve(NUMBER_OF_CHANNELS * NUMBER_OF_CELLS);
        graph_edges_.reserve(NUMBER_OF_CHANNELS * NUMBER_OF_CELLS);
        adjacency_list_.reserve(NUMBER_OF_CHANNELS * NUMBER_OF_CELLS);
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
                switch (point.label)
                {
                case SegmentationLabel::GROUND: {
                    ground_cloud.emplace_back(point.x, point.y, point.z);
                    break;
                }
                case SegmentationLabel::OBSTACLE: {
                    obstacle_cloud.emplace_back(point.x, point.y, point.z);
                    break;
                }
                case SegmentationLabel::DOUBT: {
                    // std::cout << "Found doubt point.\n";
                    [[fallthrough]];
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
    const float vehicle_height_;
    const float threshold_distance_m_;
    const float threshold_gradient_rad_;
    const float threshold_ground_height_m_;
    const float threshold_height_m_;
    const float height_difference_parameter_m_;

    std::vector<PointXYZIL> input_cloud_;

    // Channel based segmentation
    std::array<std::vector<PointXYZIL>, NUMBER_OF_CHANNELS> channel_points_;
    std::array<std::vector<CellBoundary>, NUMBER_OF_CHANNELS> cell_indices_;

    // Polar grid
    std::array<std::array<std::vector<PointXYZIL>, NUMBER_OF_CELLS>, NUMBER_OF_CHANNELS> polar_grid_;

    // Graph representation for MRF
    std::vector<GraphCell> graph_cells_; // Graph cell contains multiple points
    std::vector<GraphEdge> graph_edges_; // Edges connect the graph cells
    std::unordered_map<std::uint32_t, std::vector<std::uint32_t>> adjacency_list_;
    std::unordered_map<std::uint32_t, std::vector<Message>> messages_;

    // Step 1 of Loopy Belief Propagation
    void initializeMessages()
    {
        // Initialize messages with uniform distribution
        for (const auto &[node_index, neighbour_nodes] : adjacency_list_)
        {
            messages_[node_index] = std::vector<Message>(neighbour_nodes.size(), {0.0F, 0.0F});
        }
    }

    // Step 2 of Loopy Belief Propagation
    void updateMessages()
    {
        // Parameters for message passing - TODO: define elsewhere
        static constexpr float damping_factor{0.5F};       // Between 0 and 1
        static constexpr std::uint32_t max_iterations{4U}; // Maximum iterations for LBP
        static constexpr float delta{0.001F};              // For convergence checks

        for (std::uint32_t iteration = 0U; iteration < max_iterations; ++iteration)
        {
            // Maximum change in messages for convergence check
            float max_delta = 0.0F;

            for (const auto &[node_index, neighbour_nodes] : adjacency_list_)
            {
                for (std::uint32_t i = 0U; i < neighbour_nodes.size(); ++i)
                {
                    const auto neighbour_index = neighbour_nodes[i];
                    auto &message = messages_[node_index][i];

                    // Compute new message values
                    float new_to_ground = 0.0F;   // TODO: Compute message for ground
                    float new_to_obstacle = 0.0F; // TODO: Compute message for obstacle

                    // Damping update to avoid oscillations
                    message.to_ground = (damping_factor * message.to_ground) + (1.0F - damping_factor) * new_to_ground;
                    message.to_obstacle =
                        (damping_factor * message.to_obstacle) + (1.0F - damping_factor) * new_to_obstacle;

                    // Check for convergence
                    float message_delta =
                        std::fabs(new_to_ground - message.to_ground) + std::fabs(new_to_obstacle - message.to_obstacle);
                    max_delta = std::max(max_delta, message_delta);
                }
            }

            // If changes are below a threshold, assume convergence
            if (max_delta < delta)
            {
                break;
            }
        }
    }

    // Step 3 of Loopy Belief Propagation
    void computeBeliefs()
    {
        // Compute final beliefs for each node
        for (auto &cell : graph_cells_)
        {
            // Initialize belief based on unary potentials
            auto belief_ground = cell.unary_potential;
            auto belief_obstacle = cell.unary_potential;

            // Aggregate incoming messages from neighbours
            std::uint32_t node_index = 0U; // TODO: Node index corresponding to cell
            const auto &incoming_messages = messages_[node_index];

            for (const auto &message : incoming_messages)
            {
                belief_ground += message.to_ground;
                belief_obstacle += message.to_obstacle;
            }

            // Determine the final label based on beliefs
            cell.label = (belief_ground < belief_obstacle) ? SegmentationLabel::GROUND : SegmentationLabel::OBSTACLE;
        }
    }

    void runLoopyBeliefPropagation()
    {
        initializeMessages();
        updateMessages();
        computeBeliefs();
    }

    // Helper function to calculate the data cost
    float calculateDataCost(const GraphCell &cell)
    {
        // Category 1: No points inside the cell
        if (cell.points.empty())
        {
            return 0.0F; // The cost is zero if no information is available
        }

        // Category 2 and 3: Points inside the cell
        // Calculate the lowest and highest z values within the cell
        float lowest_z = std::numeric_limits<float>::max();
        float highest_z = std::numeric_limits<float>::lowest();
        for (const auto &point : cell.points)
        {
            if (point.z > lowest_z)
            {
                lowest_z = point.z;
            }
            if (point.z > highest_z)
            {
                highest_z = point.z;
            }
        }

        // Get the reference value g(v(i))
        float reference_height = lowest_z; // TODO: calculate this value

        // Calculate the cost based on height difference
        // TODO: Fix this formula: D(f(v(i))) = min(f(v(i)) - g(v(i)), tau), tau - truncation value
        const float cost = std::min(std::fabs(highest_z - reference_height), std::fabs(lowest_z - reference_height));
        return cost;
    }

    float calculateSmoothnessCost(const SegmentationLabel label_1, const SegmentationLabel label_2)
    {
        // TODO
        return 0.0F;
    }

    void buildGraph()
    {
        // Clear the previous graph
        graph_cells_.clear();
        graph_edges_.clear();
        adjacency_list_.clear();

        // Each cell in the polar grid becomes a node in the graph
        std::uint32_t node_index = 0U;
        for (std::uint32_t channel_index = 0U; channel_index < NUMBER_OF_CHANNELS; ++channel_index)
        {
            for (std::uint32_t cell_index = 0U; cell_index < NUMBER_OF_CELLS; ++cell_index)
            {
                // Create a node for each cell
                GraphCell node;
                node.points = polar_grid_[channel_index][cell_index];
                node.label = SegmentationLabel::DOUBT; // TODO: Calculate based on initial segmentation
                node.unary_potential = 0.0F;           // TODO: Calculate unary potential based on the criteria

                graph_cells_.push_back(node);

                // Connect each node to its four neighbouring cells
                if (cell_index > 0U)
                {
                    const std::uint32_t back_node_index = (channel_index * NUMBER_OF_CELLS) + (cell_index - 1U);

                    // TODO: Set the correct pairwise potential
                    graph_edges_.push_back({node_index, back_node_index, 0.0F});
                    adjacency_list_[node_index].push_back(back_node_index);
                }

                if (cell_index < (NUMBER_OF_CELLS - 1U))
                {
                    const std::uint32_t front_node_index = (channel_index * NUMBER_OF_CELLS) + (cell_index + 1U);

                    // TODO: Set the correct pairwise potential
                    graph_edges_.push_back({node_index, front_node_index, 0.0F});
                    adjacency_list_[node_index].push_back(front_node_index);
                }

                // Left and right neighbour nodes wrap around
                const std::uint32_t left_channel_index =
                    (channel_index == 0U) ? (NUMBER_OF_CHANNELS - 1U) : (channel_index - 1U);

                const std::uint32_t right_channel_index =
                    (channel_index == (NUMBER_OF_CHANNELS - 1U)) ? 0U : (channel_index + 1U);

                const std::uint32_t left_node_index = (left_channel_index * NUMBER_OF_CELLS) + cell_index;

                const std::uint32_t right_node_index = (right_channel_index * NUMBER_OF_CELLS) + cell_index;

                graph_edges_.push_back({node_index, left_node_index, 0.0F});
                graph_edges_.push_back({node_index, right_node_index, 0.0F});

                adjacency_list_[node_index].push_back(left_node_index);
                adjacency_list_[node_index].push_back(right_node_index);

                // Increment the node index for the next cell
                ++node_index;
            }
        }

        std::cout << "Number of elements in adjacency list: " << adjacency_list_.size() << std::endl;
    }

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

        // Sort points in ascending radial order
        for (std::uint32_t channel_index = 0U; channel_index < NUMBER_OF_CHANNELS; ++channel_index)
        {
            auto &channel = channel_points_[channel_index];

            // Sort points in by distance from the sensor
            std::sort(channel.begin(), channel.end(), [](const PointXYZIL &p1, const PointXYZIL &p2) noexcept -> bool {
                return ((p1.x * p1.x) + (p1.y * p1.y)) < ((p2.x * p2.x) + (p2.y * p2.y));
            });

            // Populate polar grid cells
            auto &grid_channel = polar_grid_[channel_index];

            // Clean old channel's points
            for (auto &polar_grid_cell : grid_channel)
            {
                polar_grid_cell.clear();
            }

            // Add new points to the polar grid
            for (auto &point : channel)
            {
                // Distance of the point from the origin
                const float point_distance = std::sqrt(point.x * point.x + point.y * point.y);

                // Calculate the radial cell index for this point
                const std::uint32_t cell_index =
                    std::min(static_cast<std::uint32_t>(point_distance / CELL_RESOLUTION_M), NUMBER_OF_CELLS - 1U);

                grid_channel[cell_index].push_back(point);
            }
        }

        // Build the adjacency matrix
        // buildGraph();

        // TODO: Run Loopy Belief Propagation
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

    void correctDoubtPoints(std::vector<PointXYZIL> &channel, const std::uint32_t last_index,
                            const SegmentationLabel label)
    {
        for (std::int32_t j = static_cast<std::int32_t>(last_index) - 1; j >= 0; --j)
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

        // std::for_each(std::execution::par, channel_points_.begin(), channel_points_.end(),
        //               [this](auto &channel) -> void { segmentChannel(channel); });
    }

    void channelBasedMarkovRandomFieldAndLoopyBeliefPropagation()
    {
        // https://nbviewer.org/github/krashkov/Belief-Propagation/blob/master/4-ImplementationBP.ipynb
        // https://nghiaho.com/?page_id=1366
    }
};
} // namespace lidar_processing

#endif // CHANNEL_BASED_GROUND_SEGMENTER