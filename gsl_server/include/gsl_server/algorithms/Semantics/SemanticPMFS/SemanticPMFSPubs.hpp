#pragma once
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>

namespace GSL::SemanticPMFS_internal
{
    struct PublishersAndSubscribers
    {
        PMFS_internal::GMRFWind gmrfWind;
        PMFS_internal::GroundTruthWind groundTruthWind;
    };
}