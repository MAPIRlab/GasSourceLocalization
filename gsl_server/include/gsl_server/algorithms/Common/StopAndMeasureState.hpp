#pragma once

#include <gsl_server/algorithms/Common/GSLState.hpp>
#include <vector>
#include <rclcpp/time.hpp>

namespace GSL
{
    class StopAndMeasureState : public State
    {
    public:
        StopAndMeasureState(Algorithm* _algorithm);
        virtual void OnEnterState(State* previousState) override;
        virtual void OnExitState(State* nextState) override;
        virtual void OnUpdate() override;

        double average_concentration();
        double average_windDirection();
        double average_windSpeed();
        virtual void addGasReading(double concentration);
        virtual void addWindReading(double speed, double direction);

    protected:
        double measure_time; // how long to measure for, in seconds
        rclcpp::Time time_stopped;

    private:
        std::vector<float> gas_v;
        std::vector<float> windSpeed_v;
        std::vector<float> windDirection_v;
    };
} // namespace GSL