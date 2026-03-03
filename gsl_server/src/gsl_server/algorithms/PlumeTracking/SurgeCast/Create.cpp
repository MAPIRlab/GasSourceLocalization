#include "Create.hpp"
#include "SurgeCast.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSurgeCast(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<SurgeCast>(node);
    }
} // namespace GSL