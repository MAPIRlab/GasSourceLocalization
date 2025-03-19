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

        double average_concentration(); //average of all the readings since we entered the state
        double average_windDirection(); //average of all the readings since we entered the state
        double average_windSpeed(); //average of all the readings since we entered the state
        virtual void addGasReading(double concentration); //called from the sensor callback
        virtual void addWindReading(double speed, double direction); //called from the sensor callback

    protected:
        double measure_time; // how long to measure for, in seconds
        rclcpp::Time time_stopped;

    private:
        std::vector<float> gas_v;
        std::vector<float> windSpeed_v;
        std::vector<float> windDirection_v;
    };
} // namespace GSL