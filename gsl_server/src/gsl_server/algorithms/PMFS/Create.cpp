#include "Create.hpp"
#include "PMFS.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreatePMFS(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<PMFS>(node);
    }
} // namespace GSL