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

// Default color (r, g, b) = (0, 0, 0)
struct Color
{
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
    std::uint8_t a; // 255 - opaque, 0 - transparent
    explicit Color(std::uint8_t _r = 0, std::uint8_t _g = 0, std::uint8_t _b = 0, std::uint8_t _a = 255)
        : r(_r), g(_g), b(_b), a(_a){};
};

Color GREY_COLOR(220, 220, 220);
Color GREEN_COLOR(0, 255, 0);
Color RED_COLOR(255, 0, 0);
Color BLUE_COLOR(0, 0, 255);
Color YELLOW_COLOR(255, 255, 0);
Color ORANGE_COLOR(255, 165, 0);
Color CYAN_COLOR(0, 255, 255);
Color BROWN_COLOR(172, 90, 0);
Color MAGENTA_COLOR(255, 0, 255);

std::uint32_t getRGBAFromColor(std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255)
{
    std::uint32_t rgba = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    return rgba;
}

std::uint32_t GREY_COLOR_RGBA = getRGBAFromColor(GREY_COLOR.r, GREY_COLOR.g, GREY_COLOR.b);
std::uint32_t GREEN_COLOR_RGBA = getRGBAFromColor(GREEN_COLOR.r, GREEN_COLOR.g, GREEN_COLOR.b);
std::uint32_t RED_COLOR_RGBA = getRGBAFromColor(RED_COLOR.r, RED_COLOR.g, RED_COLOR.b);
std::uint32_t BLUE_COLOR_RGBA = getRGBAFromColor(BLUE_COLOR.r, BLUE_COLOR.g, BLUE_COLOR.b);
std::uint32_t YELLOW_COLOR_RGBA = getRGBAFromColor(YELLOW_COLOR.r, YELLOW_COLOR.g, YELLOW_COLOR.b);
std::uint32_t ORANGE_COLOR_RGBA = getRGBAFromColor(ORANGE_COLOR.r, ORANGE_COLOR.g, ORANGE_COLOR.b);
std::uint32_t CYAN_COLOR_RGBA = getRGBAFromColor(CYAN_COLOR.r, CYAN_COLOR.g, CYAN_COLOR.b);
std::uint32_t BROWN_COLOR_RGBA = getRGBAFromColor(BROWN_COLOR.r, BROWN_COLOR.g, BROWN_COLOR.b);
std::uint32_t MAGENTA_COLOR_RGBA = getRGBAFromColor(MAGENTA_COLOR.r, MAGENTA_COLOR.g, MAGENTA_COLOR.b);

struct EnumClassHash
{
    template <typename T> std::size_t operator()(T t) const
    {
        return static_cast<std::size_t>(t);
    }
};

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