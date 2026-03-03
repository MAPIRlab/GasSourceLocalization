#include "Create.hpp"
#include "GrGSL.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateGrGSL(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<GrGSL>(node);
    }
} // namespace GSL