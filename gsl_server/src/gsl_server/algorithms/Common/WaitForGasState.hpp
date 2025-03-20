#pragma once
#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <rclcpp/time.hpp>

namespace GSL
{
    // Does nothing. As soon as a gas reading above the threshold arrives, moves to StopAndMeasure
    class WaitForGasState : public State
    {
    public:
        WaitForGasState(Algorithm* _algorithm);

        void addMeasurement(double concentration);
        void OnUpdate() override;

    protected:
        void OnEnterState(State* previous) override;

    protected:
        float maxWaitTime;
        rclcpp::Time startTime;
    };
} // namespace GSL