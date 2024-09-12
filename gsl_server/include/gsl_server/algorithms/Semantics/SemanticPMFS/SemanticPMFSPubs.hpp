#pragma once
#include <gsl_server/algorithms/PMFS/internal/PublishersAndSubscribers.hpp>

namespace GSL::SemanticPMFS_internal
{
    struct PublishersAndSubscribers
    {
        PMFS_internal::PublishersAndSubscribers pmfsPubs;
        //...

        PublishersAndSubscribers(std::shared_ptr<rclcpp::Clock> clock) : pmfsPubs(clock) {}
    };
}