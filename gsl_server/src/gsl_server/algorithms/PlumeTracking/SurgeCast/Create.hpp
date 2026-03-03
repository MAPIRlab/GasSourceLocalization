#pragma once

#include <memory>
#include <rclcpp/node.hpp>

namespace GSL
{
    std::shared_ptr<class Algorithm> CreateSurgeCast(std::shared_ptr<rclcpp::Node> node);
}