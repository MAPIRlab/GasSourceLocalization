#pragma once

#include "gsl_server/algorithms/Common/Grid2D.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"
#include "gsl_server/algorithms/Common/Utils/RosUtils.hpp"
#include "gsl_server/core/ros_typedefs.hpp"
#include <fstream>

namespace GSL::SemanticsResults
{

    static std::ofstream progressionFile;

    inline void InitFile(const std::string& progressionFileName, Vector2 sourcePosition)
    {
        progressionFile.open(progressionFileName, std::ios_base::app);
        progressionFile << fmt::format("\n!{} {}\n", sourcePosition.x, sourcePosition.y);
        progressionFile << "#...............................\n";
        progressionFile << "#expectedOlfOnly; modeOlfOnly; errorOlfOnly; varianceOlfOnly; expectedBoth; modeBoth; errorBoth; varianceBoth\n";
        progressionFile << "#-------------------------------\n";
        progressionFile.flush();
    }

    inline void LogResult(Grid2D<double> olfactionOnly, Grid2D<double> withSemantics, Vector2 sourcePosition, Vector2 colorLimits)
    {
        std::vector<ColorRGBA> colors(olfactionOnly.data.size());
        for (int i = 0; i < olfactionOnly.data.size(); i++)
            colors[i] = Utils::valueToColor(olfactionOnly.data.at(i),
                                            colorLimits.x,
                                            colorLimits.y,
                                            Utils::valueColorMode::Logarithmic);
        Utils::publishDebugMarkers(Grid2D<ColorRGBA>(colors, olfactionOnly.occupancy, olfactionOnly.metadata), "sourceOlfactionOnly");

        Vector2 expecOlfOnly = Utils::ExpectedValue(olfactionOnly, 1);
        Vector2 modeOlfOnly = Utils::Mode(olfactionOnly);
        Utils::CovarianceMatrix varOlfOnly = Utils::Covariance(olfactionOnly);
        double errorOlfOnly = vmath::length(expecOlfOnly - sourcePosition);

        Vector2 expecBoth = Utils::ExpectedValue(withSemantics, 1);
        Vector2 modeBoth = Utils::Mode(withSemantics);
        Utils::CovarianceMatrix varBoth = Utils::Covariance(withSemantics);
        double errorBoth = vmath::length(expecBoth - sourcePosition);
        progressionFile << fmt::format("{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t",
                                       expecOlfOnly.x, expecOlfOnly.y, modeOlfOnly.x, modeOlfOnly.y, errorOlfOnly, varOlfOnly.x, varOlfOnly.y, varOlfOnly.covariance);
        progressionFile << fmt::format("{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\t{:.2f}\n",
                                       expecBoth.x, expecBoth.y, modeBoth.x, modeBoth.y, errorBoth, varBoth.x, varBoth.y, varBoth.covariance);
        progressionFile.flush();

        Utils::publishDebugSingleMarker(vmath::WithZ(expecOlfOnly, 0.0),
                                        Utils::create_color(1, 0, 0, 1),
                                        "EXPECTED_OLFACTION");
        Utils::publishDebugSingleMarker(vmath::WithZ(expecBoth, 0.0),
                                        Utils::create_color(1, 0, 1, 1),
                                        "EXPECTED_BOTH");
    }
} // namespace SemanticsResults