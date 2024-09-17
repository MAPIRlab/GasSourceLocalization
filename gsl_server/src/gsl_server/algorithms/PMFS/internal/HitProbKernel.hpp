#pragma once
#include <gsl_server/core/Vectors.hpp>

namespace GSL::PMFS_internal
{
    struct HitProbKernel
    {
        double angle;
        Vector2 sigma;
        float valueAt1;
    };
} // namespace GSL::PMFS_internal