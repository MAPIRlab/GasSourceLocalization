#include "Create.hpp"
#include "Spiral.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSpiral(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<Spiral>(node);
    }
} // namespace GSL