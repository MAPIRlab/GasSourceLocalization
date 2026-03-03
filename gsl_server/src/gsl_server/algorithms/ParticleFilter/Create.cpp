#include "Create.hpp"
#include "ParticleFilter.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateParticleFilter(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<ParticleFilter>(node);
    }
} // namespace GSL