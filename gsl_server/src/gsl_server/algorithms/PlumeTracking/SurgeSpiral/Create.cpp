#include "Create.hpp"
#include "SurgeSpiral.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSurgeSpiral(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<SurgeSpiral>(node);
    }
} // namespace GSL