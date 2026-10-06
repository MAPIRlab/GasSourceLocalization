#include "Simulation.hpp"
#include <gsl_server/algorithms/Common/Utils/Images.hpp>

void GSL::Simulation::displayImage(const Grid2D<float>& hitMap, const std::string& imageName, float raisePower)
{
    std::vector<float> hitMapCopy = hitMap.data;
    for (float& f : hitMapCopy)
        f = std::pow(f, raisePower);
    cv::Mat asImage(hitMapCopy);
    asImage = asImage.reshape(1, hitMap.metadata.dimensions.y);

    cv::Mat inColor;
    asImage.convertTo(asImage, CV_8UC1, 255);
    cv::applyColorMap(asImage, inColor, cv::COLORMAP_VIRIDIS);

    for (int j = 0; j < hitMap.metadata.dimensions.y; j++)
    {
        for (int i = 0; i < hitMap.metadata.dimensions.x; i++)
        {
            if (!hitMap.occupancyAt(i, j))
                inColor.at<cv::Vec3b>(j, i) = cv::Vec3b(80, 80, 80);
        }
    }

#if 0
        cv::flip(inColor, inColor, 0);
        inColor *= 255;
        cv::imwrite(fmt::format("{}.png", imageName), inColor);
        GSL_WARN("hitMap image saved");
#else
    cv::flip(inColor, inColor, 0);
    Utils::Image::Show(inColor, imageName);
#endif
}