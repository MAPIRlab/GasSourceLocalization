#pragma once
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct HitProbKernel
    {
        float frequency; //hit frequency that was observed

        // parameters of the gaussian used to extend the effects of the measurement to nearby cells
        // the value of the gaussian at the cell center is the amount of evidence mass that gets added to the observed frequency
        double angle;
        Vector2 sigma;
    };
} // namespace GSL::PMFS_internal