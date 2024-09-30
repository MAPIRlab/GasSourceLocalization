#pragma once

#include <rclcpp_action/rclcpp_action.hpp>

#ifdef USE_NAV_ASSISTANT

#include <nav_assistant_msgs/srv/make_plan.hpp>
#include <nav_assistant_msgs/action/nav_assistant.hpp>
using NavigateToPose = nav_assistant_msgs::action::NavAssistant;
using MakePlan = nav_assistant_msgs::srv::MakePlan;

#else

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using MakePlan = nav2_msgs::action::ComputePathToPose;

#endif

using NavigationClient = rclcpp_action::Client<NavigateToPose>::SharedPtr;