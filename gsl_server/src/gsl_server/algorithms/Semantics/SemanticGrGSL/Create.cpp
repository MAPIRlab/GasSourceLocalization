#include "Create.hpp"
#include "SemanticGrGSL.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSemanticGrGSL(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<SemanticGrGSL>(node);
    }
} // namespace GSL