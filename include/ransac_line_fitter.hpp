#ifndef RANSAC_LINE_FITTER
#define RANSAC_LINE_FITTER

#include <algorithm>
#include <array>
#include <cstdint>
#include <eigen3/Eigen/Dense>

namespace lidar_processing
{
template <std::size_t MaxPoints> class RANSACLineFitter final
{
  public:
    constexpr RANSACLineFitter()
    {
    }

    // Method to add a point to the buffer
    inline void addPoint(float x, float y) noexcept
    {
        point_buffer_[current_buffer_index_++] = x;
        point_buffer_[current_buffer_index_++] = y;
    }

    // Reset buffer
    inline void resetBuffer() noexcept
    {
        current_buffer_index_ = 0U;
    }

    // Get the number of points
    inline std::uint32_t numberOfPoints() const noexcept
    {
        return (current_buffer_index_ / 2U);
    }

    // Fit
    Eigen::RowVector3f fit(float pivot_x, float pivot_y, float max_line_gradient = 0.15F,
                           float orthogonal_distance_threshold = 0.1F)
    {
        Eigen::Map<Eigen::Matrix<float, MaxPoints, 2, Eigen::RowMajor>> points_map{point_buffer_.data()};
        const Eigen::RowVector2f pivot_xy{pivot_x, pivot_y};

        Eigen::RowVector3f best_line{};
        std::uint32_t max_inliers = 0U;

        // For each point
        for (std::uint32_t point_index = 0U; point_index < numberOfPoints(); ++point_index)
        {
            const auto &point_xy = points_map.row(point_index);

            // Calculate the line coefficients
            float a = pivot_xy.y() - point_xy.y();
            float b = point_xy.x() - pivot_xy.x();

            // Normalize
            const float denom = std::sqrt(a * a + b * b);

            // Check for very small denominator and slope constraint
            if ((denom < 1e-4F) || (std::fabs(-a / b) > max_line_gradient))
            {
                continue;
            }

            const float norm = 1.0F / denom;
            a *= norm;
            b *= norm;
            const float c = (pivot_xy.x() * point_xy.y() - point_xy.x() * pivot_xy.y()) * norm;

            // Count inliers using optimized distance calculation: ax + by + c = 0
            std::uint32_t inliers = 0U;
            for (std::uint32_t idx = 0U; idx < numberOfPoints(); ++idx)
            {
                const auto &other_point = points_map.row(idx);
                const float distance = std::abs(a * other_point.x() + b * other_point.y() + c);
                if (distance <= orthogonal_distance_threshold)
                {
                    ++inliers;
                }
            }

            // Update best line if more inliers are found
            if (inliers > max_inliers)
            {
                max_inliers = inliers;
                best_line.x() = a;
                best_line.y() = b;
                best_line.z() = c;
            }
        }

        return best_line;
    }

  private:
    std::array<float, MaxPoints * 2> point_buffer_;
    std::uint32_t current_buffer_index_;
};
} // namespace lidar_processing

#endif // RANSAC_LINE_FITTER