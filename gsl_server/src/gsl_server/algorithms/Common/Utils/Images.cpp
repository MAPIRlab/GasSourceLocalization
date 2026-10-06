#include "Images.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <gsl_server/algorithms/Common/Grid2D.hpp>
#include <gsl_server/algorithms/Common/Utils/Math.hpp>

namespace GSL::Utils::Image
{
    void Show(const cv::Mat& mat, std::string name)
    {
        cv::Mat resized;
        cv::resize(mat, resized, cv::Size(mat.size[1] * 10, mat.size[0] * 10), 0, 0, cv::INTER_NEAREST);
        cv::imshow(name, resized);
        cv::setWindowProperty(name, cv::WND_PROP_TOPMOST, 1); // force focus
        while (cv::getWindowProperty(name, cv::WindowPropertyFlags::WND_PROP_VISIBLE) && cv::waitKey(30) == -1)
            ;
        cv::destroyWindow(name);
    }


    void Blur(std::vector<float>& hitMap, float blurSigma, GSL::Grid2D<GSL::Occupancy> occupancy, std::optional<BlurMask>& blurredMask)
    {
        if (blurSigma == 0)
            return;
        cv::Mat asImage(hitMap, false); // copyData=false, so changes to the matrix will affect the hitMap vector
        asImage = asImage.reshape(1, occupancy.metadata.dimensions.y);

        cv::GaussianBlur(asImage, asImage, cv::Size(0, 0), blurSigma, blurSigma);

        // divide by the blurred mask to correct the edges always getting lower
        if (!blurredMask || blurredMask->sigma != blurSigma)
        {
            blurredMask.emplace();
            blurredMask->sigma = blurSigma;
            cv::Mat freeSpaceMask(
                cv::Size(occupancy.metadata.dimensions.x, occupancy.metadata.dimensions.y),
                CV_32F,
                cv::Scalar(0, 0, 0));

            for (int j = 0; j < occupancy.metadata.dimensions.y; j++)
            {
                for (int i = 0; i < occupancy.metadata.dimensions.x; i++)
                {
                    if (occupancy.occupancyAt(i, j))
                        freeSpaceMask.at<float>(j, i) = 1;
                }
            }

            cv::GaussianBlur(freeSpaceMask, blurredMask->mask, cv::Size(0, 0), blurSigma, blurSigma);
        }

        for (int i = 0; i < occupancy.metadata.dimensions.y; i++)
            for (int j = 0; j < occupancy.metadata.dimensions.x; j++)
            {
                if (!occupancy.occupancyAt(j, i))
                    asImage.at<float>(i, j) = 0;
                else if (blurredMask->mask.at<float>(i, j) > 0)
                    asImage.at<float>(i, j) = GSL::Utils::clamp(asImage.at<float>(i, j) / blurredMask->mask.at<float>(i, j), 0, 1);
            }
    }
} // namespace Utils::Image