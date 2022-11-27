#pragma once
#ifndef POINT_LABELS_HPP_
#define POINT_LABELS_HPP_

#include <cinttypes>
#include <unordered_map>

namespace lidar_processing
{
enum class Labels
{
    GROUND = 0,
    OBSTACLE = 1
};

constexpr std::uint8_t GROUND_LABEL = static_cast<std::uint8_t>(Labels::GROUND);
constexpr std::uint8_t OBSTACLE_LABEL = static_cast<std::uint8_t>(Labels::OBSTACLE);

struct EnumClassHash
{
    template <typename T> std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

// Default color (r, g, b) = (0, 0, 0)
struct Color
{
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
    explicit Color(std::uint8_t _r = 0, std::uint8_t _g = 0, std::uint8_t _b = 0) : r(_r), g(_g), b(_b){};
};

Color GREY_COLOR(220, 220, 220);
Color GREEN_COLOR(0, 255, 0);

std::unordered_map<std::uint8_t, Color, EnumClassHash> POINT_CLOUD_LABELS = {{GROUND_LABEL, GREY_COLOR},     // grey
                                                                             {OBSTACLE_LABEL, GREEN_COLOR}}; // green

// converts rgba to Color(r, g, b)
Color getColorFromRGB(std::uint32_t rgba)
{
    std::uint8_t r = (rgba >> 16) & 0x0000ff;
    std::uint8_t g = (rgba >> 8) & 0x0000ff;
    std::uint8_t b = (rgba)&0x0000ff;
    Color color(r, g, b);
    return color;
}

// returns one of the labels from point cloud Labels (uint8_t)
std::int16_t getLabelFromColor(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
    for (const auto &label_pair : POINT_CLOUD_LABELS)
    {
        const auto &label_color = label_pair.second;
        if (label_color.r == r && label_color.g == g && label_color.b == b)
        {
            return static_cast<std::int16_t>(label_pair.first);
        }
    }
    return -1;
}
} // namespace lidar_processing

#endif // POINT_LABELS_HPP_