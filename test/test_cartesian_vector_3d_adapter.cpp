#include "cartesian_vector_3d_adapter.hpp"
#include <gtest/gtest.h>
#include <initializer_list>

using namespace containers;

TEST(CartesianVector3DAdapterTest, ConstructEmpty)
{
    auto data = std::make_shared<std::vector<float>>();
    CartesianVector3DAdapter adapter(data);
    EXPECT_EQ(adapter.size(), 0);
}

TEST(CartesianVector3DAdapterTest, AddPointsAndAccess)
{
    auto data = std::make_shared<std::vector<float>>();
    CartesianVector3DAdapter adapter(data);
    adapter.emplace_back(1.0f, 2.0f, 3.0f);
    adapter.emplace_back(4.0f, 5.0f, 6.0f);

    EXPECT_EQ(adapter.size(), 2);

    auto point = adapter[0];
    EXPECT_FLOAT_EQ(point.x(), 1.0f);
    EXPECT_FLOAT_EQ(point.y(), 2.0f);
    EXPECT_FLOAT_EQ(point.z(), 3.0f);

    adapter[0].set(10.0f, 20.0f, 30.0f);
    EXPECT_FLOAT_EQ(adapter[0].x(), 10.0f);
    EXPECT_FLOAT_EQ(adapter[0].y(), 20.0f);
    EXPECT_FLOAT_EQ(adapter[0].z(), 30.0f);
}

TEST(CartesianVector3DAdapterTest, ResizeAndAccessOutOfBounds)
{
    auto data = std::make_shared<std::vector<float>>();
    CartesianVector3DAdapter adapter(data);
    adapter.reserve(100);
    EXPECT_EQ(adapter.capacity(), 100);
    adapter.resize(2);
    EXPECT_EQ(adapter.size(), 2);

    EXPECT_THROW(adapter.at(2), std::out_of_range);
}

TEST(CartesianVector3DAdapterTest, DirectDimensionAccess)
{
    auto data = std::make_shared<std::vector<float>>(std::initializer_list<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    CartesianVector3DAdapter adapter(data);

    EXPECT_FLOAT_EQ(adapter(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(adapter(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(adapter(0, 2), 3.0f);

    adapter(1, 0) = 10.0f;
    adapter(1, 1) = 20.0f;
    adapter(1, 2) = 30.0f;

    EXPECT_FLOAT_EQ(adapter(1, 0), 10.0f);
    EXPECT_FLOAT_EQ(adapter(1, 1), 20.0f);
    EXPECT_FLOAT_EQ(adapter(1, 2), 30.0f);
}

TEST(CartesianVector3DAdapterTest, BoundsCheckedAccess)
{
    auto data = std::make_shared<std::vector<float>>(std::initializer_list<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    CartesianVector3DAdapter adapter(data);

    EXPECT_FLOAT_EQ(adapter.at(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(adapter.at(0, 1), 2.0f);
    EXPECT_FLOAT_EQ(adapter.at(0, 2), 3.0f);

    EXPECT_THROW(adapter.at(2, 0), std::out_of_range); // Should throw because index 6 is out of bounds
    EXPECT_THROW(adapter.at(0, 3), std::out_of_range); // Should throw because dim 3 is out of data range
    EXPECT_NO_THROW(adapter.at(1, 2));
}

TEST(CartesianVector3DAdapterTest, AssignFunctionality)
{
    auto data = std::make_shared<std::vector<float>>();
    CartesianVector3DAdapter adapter(data);
    adapter.assign(3, 1.0f, 2.0f, 3.0f);

    ASSERT_EQ(adapter.size(), 3);
    EXPECT_FLOAT_EQ(adapter(0, 0), 1.0f);
    EXPECT_FLOAT_EQ(adapter(2, 2), 3.0f);
}

TEST(CartesianVector3DAdapterTest, ClearFunctionality)
{
    auto data = std::make_shared<std::vector<float>>(std::initializer_list<float>{1.0f, 2.0f, 3.0f});
    CartesianVector3DAdapter adapter(data);
    adapter.clear();
    EXPECT_EQ(adapter.size(), 0);
    EXPECT_TRUE(adapter.empty());
    EXPECT_EQ(data->size(), 0);
}

TEST(CartesianVector3DAdapterTest, PopBackFunctionality)
{
    auto data = std::make_shared<std::vector<float>>(std::initializer_list<float>{1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f});
    CartesianVector3DAdapter adapter(data);
    adapter.pop_back();
    EXPECT_EQ(adapter.size(), 1);
}

TEST(CartesianVector3DAdapterTest, CapacityTest)
{
    auto data = std::make_shared<std::vector<float>>(10);
    CartesianVector3DAdapter adapter(data);
    EXPECT_EQ(adapter.capacity(), 3); // Capacity is the number of complete points the vector can hold
    adapter.resize(2);
    EXPECT_GE(adapter.capacity(), 2);
}
