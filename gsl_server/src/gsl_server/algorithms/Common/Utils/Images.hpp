#pragma once
#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include <opencv2/core/mat.hpp>

namespace GSL::Utils::Image
{
    struct BlurMask
    {
        float sigma = 0.0;
        cv::Mat mask;
    };


    void Show(const cv::Mat& mat, std::string name);
    void Blur(std::vector<float>& hitMap, float blurSigma, GSL::Grid2D<GSL::Occupancy> occupancy, std::optional<BlurMask>& blurredMask);
}