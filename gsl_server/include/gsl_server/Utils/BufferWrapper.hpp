#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class BufferWrapper
{
public:
    BufferWrapper(rclcpp::Clock::SharedPtr clock) : buffer(clock), tf_listener(buffer)
    {
    }

    tf2_ros::Buffer buffer;

private:
    tf2_ros::TransformListener tf_listener;
};