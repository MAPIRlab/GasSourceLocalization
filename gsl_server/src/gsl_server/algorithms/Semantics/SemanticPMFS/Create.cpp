#include "Create.hpp"
#include "SemanticPMFS.hpp"

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSemanticPMFS(std::shared_ptr<rclcpp::Node> node)
    {
        return std::make_shared<SemanticPMFS>(node);
    }
} // namespace GSL