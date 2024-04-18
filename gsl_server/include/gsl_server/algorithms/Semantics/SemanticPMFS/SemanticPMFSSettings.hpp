#pragma once
#include <gsl_server/algorithms/PMFS/internal/Settings.hpp>

namespace GSL::SemanticPMFS_internal
{
    struct Settings
    {
        PMFS_internal::HitProbabilitySettings hitProbability;
        PMFS_internal::SimulationSettings simulation;
        //movement;
        //visualization;
    };
}