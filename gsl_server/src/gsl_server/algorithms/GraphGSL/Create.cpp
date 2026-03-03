#include "Create.hpp"
#include "GraphGSL.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateGraphGSL(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<GraphGSL>(node);
    }
} // namespace GSL