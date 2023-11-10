#pragma once

#include <gsl_server/algorithms/Common/GSLState.h>
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
        double average_wind_direction();
        double average_wind_speed();
        void addGasReading(double concentration);
        void addWindReading(double speed, double direction);

    private:
        std::vector<float> gas_v;
        std::vector<float> windSpeed_v;
        std::vector<float> windDirection_v;

        double measure_time; // how long to measure for, in seconds
        rclcpp::Time time_stopped;
    };
} // namespace GSL