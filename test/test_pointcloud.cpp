#include <gtest/gtest.h>
#include <pointcloud.hpp>

#include <cmath>

using namespace lidar_processing;

class PointCloudTest : public ::testing::Test
{
  protected:
    struct PointXYZ final
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
        PointXYZ() = default;
        PointXYZ(float x, float y, float z = 0.0F) : x(x), y(y), z(z)
        {
        }
    };

    PointCloud<PointXYZ> pointcloud_;

    void add_sample_cluster()
    {
        pointcloud_.push_into_last_cluster({1.0f, 2.0f, 3.0f});
        pointcloud_.push_into_last_cluster({4.0f, 5.0f, 6.0f});
        pointcloud_.emplace_into_last_cluster(7.0f, 8.0f, 9.0f);
    }

    std::vector<PointXYZ> get_sample_cluster()
    {
        return {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}};
    }

  public:
    friend bool operator==(const PointXYZ &a, const PointXYZ &b) noexcept
    {
        static constexpr float EPSILON = 1e-6;
        return (std::fabs(a.x - b.x) < EPSILON) && (std::fabs(a.y - b.y) < EPSILON) && (std::fabs(a.z - b.z) < EPSILON);
    }
};

TEST_F(PointCloudTest, StartsEmpty)
{
    EXPECT_EQ(pointcloud_.total_points(), 0);
}

TEST_F(PointCloudTest, AddPointsToCluster)
{
    PointXYZ point = {1.0f, 2.0f, 3.0f};
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point);
    EXPECT_EQ(pointcloud_.at_cluster(0).size(), 1);
    EXPECT_TRUE(pointcloud_.at_cluster(0)[0] == point);
}

TEST_F(PointCloudTest, MultipleClusters)
{
    PointXYZ point1 = {1.0f, 2.0f, 3.0f};
    PointXYZ point2 = {4.0f, 5.0f, 6.0f};
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point1);
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point2);
    EXPECT_EQ(pointcloud_.at_cluster(0).size(), 1);
    EXPECT_EQ(pointcloud_.at_cluster(1).size(), 1);
    EXPECT_TRUE(pointcloud_.at_cluster(0)[0] == point1);
    EXPECT_TRUE(pointcloud_.at_cluster(1)[0] == point2);
}

TEST_F(PointCloudTest, ClearPointCloud)
{
    PointXYZ point = {1.0f, 2.0f, 3.0f};
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point);
    pointcloud_.clear();
    EXPECT_EQ(pointcloud_.total_points(), 0);
}

TEST_F(PointCloudTest, AccessOutOfBounds)
{
    pointcloud_.increment_cluster();
    EXPECT_THROW(pointcloud_.at_cluster(0).at(0), std::runtime_error);
}

TEST_F(PointCloudTest, PopLastCluster)
{
    PointXYZ point1 = {1.0f, 2.0f, 3.0f};
    PointXYZ point2 = {4.0f, 5.0f, 6.0f};
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point1);
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster(point2);
    pointcloud_.pop_last_cluster();
    EXPECT_EQ(pointcloud_.total_points(), 1);
    EXPECT_TRUE(pointcloud_.at_cluster(0).at_point(0) == point1);
}

TEST_F(PointCloudTest, ForwardIterator)
{
    add_sample_cluster();
    auto cluster = pointcloud_.at_cluster(0);
    auto it = cluster.begin();
    const auto sample_cluster = get_sample_cluster();
    EXPECT_EQ(*it, sample_cluster[0]);
    ++it;
    EXPECT_EQ(*it, sample_cluster[1]);
    ++it;
    EXPECT_EQ(*it, sample_cluster[2]);
    ++it;
    EXPECT_EQ(it, cluster.end());
}

TEST_F(PointCloudTest, ReverseIterator)
{
    add_sample_cluster();
    auto cluster = pointcloud_.at_cluster(0);
    auto rit = cluster.rbegin();
    const auto sample_cluster = get_sample_cluster();
    EXPECT_EQ(*rit, sample_cluster[2]);
    ++rit;
    EXPECT_EQ(*rit, sample_cluster[1]);
    ++rit;
    EXPECT_EQ(*rit, sample_cluster[0]);
    ++rit;
    EXPECT_EQ(rit, cluster.rend());
}

TEST_F(PointCloudTest, ConstForwardIterator)
{
    add_sample_cluster();
    const auto &constCluster = pointcloud_.at_cluster(0);
    auto cit = constCluster.cbegin();
    const auto sample_cluster = get_sample_cluster();
    EXPECT_EQ(*cit, sample_cluster[0]);
    ++cit;
    EXPECT_EQ(*cit, sample_cluster[1]);
    ++cit;
    EXPECT_EQ(*cit, sample_cluster[2]);
    ++cit;
    EXPECT_EQ(cit, constCluster.cend());
}

TEST_F(PointCloudTest, ConstReverseIterator)
{
    add_sample_cluster();
    const auto &constCluster = pointcloud_.at_cluster(0);
    auto crit = constCluster.crbegin();
    const auto sample_cluster = get_sample_cluster();
    EXPECT_EQ(*crit, sample_cluster[2]);
    ++crit;
    EXPECT_EQ(*crit, sample_cluster[1]);
    ++crit;
    EXPECT_EQ(*crit, sample_cluster[0]);
    ++crit;
    EXPECT_EQ(crit, constCluster.crend());
}

TEST_F(PointCloudTest, DynamicResizing)
{
    for (int i = 0; i < 1000; i++)
    {
        pointcloud_.push_into_last_cluster({1.0f * i, 2.0f * i, 3.0f * i});
    }
    EXPECT_EQ(pointcloud_.total_points(), 1000);
    const PointXYZ expected_last_point{999.0f, 1998.0f, 2997.0f};
    EXPECT_TRUE(pointcloud_.at_cluster(0)[999] == expected_last_point);
}

TEST_F(PointCloudTest, CorrectClusterIncrementing)
{
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster({1.0f, 2.0f, 3.0f});
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster({4.0f, 5.0f, 6.0f});
    const auto point_0 = PointXYZ{1.0f, 2.0f, 3.0f};
    EXPECT_EQ(pointcloud_.at_cluster(0)[0], point_0);
    const auto point_1 = PointXYZ{4.0f, 5.0f, 6.0f};
    EXPECT_EQ(pointcloud_.at_cluster(1)[0], point_1);
    EXPECT_EQ(pointcloud_.total_clusters(), 2);
}

TEST_F(PointCloudTest, EmptyClusterHandling)
{
    pointcloud_.increment_cluster();
    EXPECT_NO_THROW({
        auto cluster = pointcloud_.at_cluster(0);
        EXPECT_EQ(cluster.size(), 0);
    });
}

TEST_F(PointCloudTest, ExceptionSafety)
{
    EXPECT_THROW(pointcloud_.at_cluster(1), std::runtime_error);
    pointcloud_.increment_cluster();
    pointcloud_.push_into_last_cluster({1.0f, 2.0f, 3.0f});
    EXPECT_EQ(pointcloud_.total_points(), 1);
}

TEST_F(PointCloudTest, CopyConstructor)
{
    add_sample_cluster();
    const PointCloud<PointXYZ> copied_pointcloud = pointcloud_;
    const auto sample_cluster = get_sample_cluster();
    EXPECT_TRUE(copied_pointcloud.at_cluster(0)[0] == sample_cluster[0]);
}

TEST_F(PointCloudTest, MoveConstructor)
{
    add_sample_cluster();
    const PointCloud<PointXYZ> moved_pointcloud = std::move(pointcloud_);
    const auto sample_cluster = get_sample_cluster();
    EXPECT_TRUE(moved_pointcloud.at_cluster(0)[0] == sample_cluster[0]);
    EXPECT_EQ(pointcloud_.total_points(), 0);
}
