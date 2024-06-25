#pragma once
#include <cmath>
#include <std_msgs/msg/color_rgba.hpp>

namespace GSL::Utils
{
    std_msgs::msg::ColorRGBA HSVtoRGB(float& H, float& S, float& V)
    {
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
    ColorHSV RGBtoHSV(const std_msgs::msg::ColorRGBA& rgb)
    {
        float fCMax = std::max({rgb.r, rgb.g, rgb.b});
        float fCMin = std::min({rgb.r, rgb.g, rgb.b});
        float fDelta = fCMax - fCMin;

        ColorHSV hsv;
        if (fDelta > 0)
        {
            if (fCMax == rgb.r)
            {
                hsv.H = 60 * (fmod(((rgb.g - rgb.b) / fDelta), 6));
            }
            else if (fCMax == rgb.g)
            {
                hsv.H = 60 * (((rgb.b - rgb.r) / fDelta) + 2);
            }
            else if (fCMax == rgb.b)
            {
                hsv.H = 60 * (((rgb.r - rgb.g) / fDelta) + 4);
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
} // namespace GSL::Utils