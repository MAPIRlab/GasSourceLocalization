#pragma once
#include <cmath>
#include <std_msgs/msg/color_rgba.hpp>

namespace GSL::Utils::Colors
{
    using ColorRGBA = std_msgs::msg::ColorRGBA;
    static ColorRGBA HSVtoRGB(float H, float S, float V)
    {
        H *= 360;
        float fC = V * S; // Chroma
        float fHPrime = fmod(H / 60.0, 6);
        float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
        float fM = V - fC;
        std_msgs::msg::ColorRGBA rgb;
        rgb.a = 1;

        if (0 <= fHPrime && fHPrime < 1)
        {
            rgb.r = fC;
            rgb.g = fX;
            rgb.b = 0;
        }
        else if (1 <= fHPrime && fHPrime < 2)
        {
            rgb.r = fX;
            rgb.g = fC;
            rgb.b = 0;
        }
        else if (2 <= fHPrime && fHPrime < 3)
        {
            rgb.r = 0;
            rgb.g = fC;
            rgb.b = fX;
        }
        else if (3 <= fHPrime && fHPrime < 4)
        {
            rgb.r = 0;
            rgb.g = fX;
            rgb.b = fC;
        }
        else if (4 <= fHPrime && fHPrime < 5)
        {
            rgb.r = fX;
            rgb.g = 0;
            rgb.b = fC;
        }
        else if (5 <= fHPrime && fHPrime < 6)
        {
            rgb.r = fC;
            rgb.g = 0;
            rgb.b = fX;
        }
        else
        {
            rgb.r = 0;
            rgb.g = 0;
            rgb.b = 0;
        }

        rgb.r += fM;
        rgb.g += fM;
        rgb.b += fM;

        return rgb;
    }

    struct ColorHSV
    {
        float H, S, V;
    };
    static ColorHSV RGBtoHSV(const ColorRGBA& rgb)
    {
        float r = rgb.r * 255;
        float g = rgb.g * 255;
        float b = rgb.b * 255;
        float fCMax = std::max({r, g, b});
        float fCMin = std::min({r, g, b});
        float fDelta = fCMax - fCMin;

        ColorHSV hsv;
        if (fDelta > 0)
        {
            if (fCMax == r)
            {
                hsv.H = 60 * (fmod(((g - b) / fDelta), 6));
            }
            else if (fCMax == g)
            {
                hsv.H = 60 * (((b - r) / fDelta) + 2);
            }
            else if (fCMax == b)
            {
                hsv.H = 60 * (((r - g) / fDelta) + 4);
            }

            if (fCMax > 0)
            {
                hsv.S = fDelta / fCMax;
            }
            else
            {
                hsv.S = 0;
            }

            hsv.V = fCMax;
        }
        else
        {
            hsv.H = 0;
            hsv.S = 0;
            hsv.V = fCMax;
        }

        if (hsv.H < 0)
        {
            hsv.H = 360 + hsv.H;
        }
        return hsv;
    }

    static ColorRGBA lerp(const ColorRGBA& a, const ColorRGBA& b, float t)
    {
        ColorRGBA v;
        v.r = std::lerp(a.r, b.r, t);
        v.g = std::lerp(a.g, b.g, t);
        v.b = std::lerp(a.b, b.b, t);
        v.a = std::lerp(a.a, b.a, t);
        return v;
    }

} // namespace GSL::Utils