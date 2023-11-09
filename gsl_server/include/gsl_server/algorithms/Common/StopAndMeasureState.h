#pragma once

#include <gsl_server/algorithms/Common/GSLState.h>

namespace GSL
{
    class StopAndMeasureState : public State
    {
    public:
        virtual void OnEnterState(State* previousState) override;
        virtual void OnExitState(State* nextState) override;
        virtual void OnUpdate() override;

        double average_concentration();
        double average_wind_direction();
        double average_wind_speed();

    private:
        std::vector<float> gas_v;
        std::vector<float> windS_v;
        std::vector<float> windD_v;

        rclcpp::Time time_stopped;

        float get_average_vector(std::vector<float> const& v)
        {
            size_t length = v.size();
            float sum = 0.0;
            for (std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
                sum += *i;

            return sum / length;
        }
    };
} // namespace GSL